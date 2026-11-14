#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""auto_launch/install_service.py — 把看门狗装成 systemd 系统服务（开机自启）

做的事情很少，但每一条都为了「机器人上电即工作、且出问题能查」：
  1. 从同目录 auto_launch.conf 读取 [launch]/[watchdog]/[service]，生成 unit 文件
     （ExecStart 使用 --python 或可用的 /usr/bin/python3，缺省回退当前解释器）；
  2. 安装前先跑一次 watchdog.py --check 做静态自检（配置/路径/权限），失败则中止；
  3. 安装前清理目标位置的「屏蔽残留」：指向 /dev/null 的符号链接（systemctl mask 的产物）
     与 0 字节文件（**systemd 把空 unit 文件同样视为 masked**）——两者都会让服务永远起不来；
  4. sudo install 到 /etc/systemd/system/ → **sync 落盘** → 回读校验内容 → daemon-reload →
     enable --now，最后核实真实状态（is-enabled / is-active），失败时给出可操作的
     排查入口；
  5. 支持 status / restart / stop / start / logs / print / uninstall。

为什么必须 sync + 回读校验（真机踩过的坑）：
  装完立刻断电，ext4（delayed allocation，默认 5s 提交窗口）可能只落盘了 inode 元数据
  （权限/属主/mtime 都在），数据块却还没写；日志恢复后文件变成 0 字节，而 systemd 对
  0 字节 unit 文件按 masked 处理 —— 表现就是「装完明明是好的，重启后 systemctl status
  显示 masked」。本脚本在安装后 sync 并回读比对，把这种状态当场暴露出来。

用法：
  ./install_service.py                      # 安装并启动（默认动作，需要 sudo）
  ./install_service.py --dry-run            # 只打印 unit 文件与将要执行的命令，不动系统
  ./install_service.py --no-enable          # 只安装 unit，不 enable/start（安全调试用）
  ./install_service.py --force              # 跳过 watchdog.py --check 自检
  ./install_service.py --user rm1 install   # 指定服务运行用户（默认：调用者的用户）
  ./install_service.py print                # 打印 unit 文件
  ./install_service.py status               # 状态（会指出 masked / 0 字节文件的成因）
  ./install_service.py logs                 # journalctl 最近 200 行
  ./install_service.py uninstall            # 停用并删除 unit（保留项目文件与日志）

说明：
  - 服务是**系统级**的（/etc/systemd/system），开箱即用、不依赖用户登录；
  - 服务运行用户默认是调用本脚本的用户（sudo 下取 SUDO_USER），因为它需要访问串口；
  - 生成 unit 的 Restart 策略跟随配置的 launch.restart_policy：
      always     → Restart=always（子进程正常退出也重启，串口服务自启动用）
      unexpected → Restart=on-failure
"""

import argparse
import hashlib
import os
import pwd
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from watchdog import Config, ConfigError, DEFAULT_CONFIG_NAME  # noqa: E402

DEFAULT_UNIT_DIR = "/etc/systemd/system"
DEFAULT_PYTHON = "/usr/bin/python3"


def log(msg):
    print(f"[install_service] {msg}", flush=True)


def is_root():
    return os.geteuid() == 0


def sudo_prefix(user=None):
    """需要提权时返回 ['sudo']（已是 root 则为空）"""
    return [] if is_root() else ["sudo"]


def detect_user(explicit=None):
    """服务运行用户：显式指定 > sudo 调用者 > 当前用户"""
    if explicit:
        return explicit
    sudo_user = os.environ.get("SUDO_USER")
    if sudo_user and sudo_user != "root":
        return sudo_user
    if is_root():
        return None   # root 直接安装但没说是哪个用户：交给调用方报错提示
    return pwd.getpwuid(os.getuid()).pw_name


def pick_python(explicit=None):
    if explicit:
        return explicit
    if os.path.exists(DEFAULT_PYTHON):
        return DEFAULT_PYTHON
    return sys.executable


def systemd_quote(text):
    """systemd 路径参数含空格或引号时加双引号"""
    if any(ch.isspace() for ch in text) or '"' in text or "\\" in text:
        escaped = text.replace("\\", "\\\\").replace('"', '\\"')
        return f'"{escaped}"'
    return text


def render_unit(cfg, user, python, watchdog_path):
    """生成 unit 文本。所有路径写绝对路径，服务不依赖登录环境。"""
    for label, path in (("解释器", python), ("看门狗", watchdog_path),
                        ("项目根", cfg.project_root), ("配置文件", cfg.path)):
        if "%" in path or "$" in path:
            raise SystemExit(f"[install_service] {label}路径包含 systemd 展开字符（% 或 $），"
                             f"请更换路径: {path}")
    restart = "always" if cfg.restart_policy == "always" else "on-failure"
    env_lines = "".join(f"Environment={systemd_quote(kv)}\n" for kv in cfg.service_environment)
    after = " ".join(cfg.service_after)
    wants = " ".join(cfg.service_wants)
    after_line = f"After={after}\n" if after else ""
    wants_line = f"Wants={wants}\n" if wants else ""
    gid = primary_gid(user)
    group_line = f"Group={gid}\n" if gid is not None else ""

    return f"""# 由 auto_launch/install_service.py 生成，请勿手工编辑（改 auto_launch.conf 后重新安装）
# 配置文件: {cfg.path}
# 运行用户: {user}
[Unit]
Description={cfg.service_description.replace('%', '%%')}
Documentation=file:{systemd_quote(os.path.join(HERE, 'README.md'))}
{after_line}{wants_line}# 不做启动频率限制：看门狗异常退出后由 systemd 拉起；退出码 2（配置错误/达到 max_restarts）不重启
StartLimitIntervalSec=0

[Service]
Type=simple
User={user}
{group_line}WorkingDirectory={systemd_quote(cfg.project_root)}
{env_lines}ExecStart={systemd_quote(python)} {systemd_quote(watchdog_path)} --config {systemd_quote(cfg.path)}
ExecReload=/bin/kill -HUP $MAINPID
Restart={restart}
RestartPreventExitStatus=2
RestartSec={cfg.service_restart_sec}
KillMode=control-group
KillSignal=SIGTERM
TimeoutStopSec={cfg.service_timeout_stop_sec}
# 需要更高调度优先级时自行打开（负值需要 CAP_SYS_NICE，可能因权限启动失败）
#Nice=-5

[Install]
WantedBy=multi-user.target
"""


def primary_gid(user):
    """用户主组 gid（不存在则 None）"""
    try:
        return pwd.getpwnam(user).pw_gid
    except KeyError:
        return None


def user_can_access(user, path, need):
    """按目标用户的 uid/gid 与权限位判断可访问性（不处理 ACL）"""
    try:
        pw = pwd.getpwnam(user)
    except KeyError:
        return False
    try:
        st = os.stat(path)
    except OSError:
        return False
    if pw.pw_uid == 0:
        return True
    if st.st_uid == pw.pw_uid:
        bits = (st.st_mode >> 6) & 0o7
    else:
        groups = set(os.getgrouplist(user, pw.pw_gid))
        if st.st_gid in groups:
            bits = (st.st_mode >> 3) & 0o7
        else:
            bits = st.st_mode & 0o7
    needed = 0
    if need & os.R_OK:
        needed |= 0o4
    if need & os.W_OK:
        needed |= 0o2
    if need & os.X_OK:
        needed |= 0o1
    return (bits & needed) == needed


def safe_user_exists(user):
    try:
        pwd.getpwnam(user)
        return True
    except KeyError:
        return False


def unit_path(name, unit_dir=DEFAULT_UNIT_DIR):
    return os.path.join(unit_dir, name)


# ── 目标路径状态：软链到 /dev/null（mask）与 0 字节文件是 systemd 里「masked」的两种来源 ──
UNIT_STATE_DESC = {
    "missing": "无",
    "file": "普通文件（会被覆盖）",
    "empty": "0 字节文件 —— systemd 把空 unit 文件当成 masked，必须先删除",
    "symlink": "符号链接 —— 多为 systemctl mask 产生的 /dev/null 屏蔽，必须先删除链接",
    "dir": "目录 —— 异常状态，需人工先删除",
}


def unit_state(path):
    if not os.path.lexists(path):
        return "missing"
    if os.path.islink(path):
        return "symlink"
    if os.path.isdir(path):
        return "dir"
    try:
        if os.path.getsize(path) == 0:
            return "empty"
    except OSError:
        return "other"
    return "file"


def file_sha256(path):
    h = hashlib.sha256()
    try:
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                h.update(chunk)
    except OSError:
        return None
    return h.hexdigest()


def run(cmd, dry=False, check=True):
    log("$ " + " ".join(cmd))
    if dry:
        return 0
    try:
        return subprocess.call(cmd)
    except FileNotFoundError as e:
        if check:
            log(f"命令不存在: {e}")
        return 127


def require(rc, what, hint=""):
    if rc != 0:
        raise SystemExit(f"[install_service] {what} 失败（退出码 {rc}）" + (f"；{hint}" if hint else ""))


def query_state(name):
    """读 systemd 的真实状态（只读，不需要 root）"""
    def _q(verb):
        try:
            out = subprocess.run(["systemctl", verb, name], capture_output=True, text=True)
            return out.stdout.strip()
        except FileNotFoundError:
            return "?"
    return _q("is-enabled"), _q("is-active")


def precheck(cfg, python, watchdog_path, dry):
    """安装前自检：配置文件 + 脚本存在 + --check 通过"""
    if not os.path.isfile(watchdog_path):
        raise SystemExit(f"[install_service] 找不到看门狗脚本: {watchdog_path}")
    if not os.path.isfile(cfg.path):
        raise SystemExit(f"[install_service] 找不到配置文件: {cfg.path}")
    if dry:
        log("dry-run：跳过 watchdog.py --check")
        return
    log("安装前自检: watchdog.py --check")
    rc = subprocess.call([python, watchdog_path, "--config", cfg.path, "--check"])
    if rc != 0:
        raise SystemExit("[install_service] 自检未通过，已中止安装"
                         "（确认无误可用 --force 跳过）")


def verify_installed_unit(dest, expected_text):
    """回读安装结果：大小 + 内容哈希都要对得上（防「装完断电变 0 字节」这类情况）"""
    got = file_sha256(dest)
    want = hashlib.sha256(expected_text.encode("utf-8")).hexdigest()
    try:
        size = os.path.getsize(dest)
    except OSError:
        size = -1
    if got == want:
        log(f"内容校验 : OK（{size} 字节，sha256 {want[:12]}…）")
        return True
    log(f"!! 内容校验失败：{dest} 实际 {size} 字节，sha256 {str(got)[:12]}…，"
        f"期望 {len(expected_text.encode('utf-8'))} 字节 / {want[:12]}…")
    if size == 0:
        log("   0 字节说明数据没落盘（ext4 delayed allocation + 装完立刻断电的典型表现；"
            "systemd 会把空 unit 当成 masked）。请重新安装，并在断电前执行一次 sync。")
    return False


def report_state(cfg, wait_sec=6.0):
    """安装后核实真实状态：is-enabled / is-active（失败给排查入口，返回非 0）"""
    name = cfg.service_name
    enabled, active = query_state(name)
    deadline = time.time() + wait_sec
    while active != "active" and time.time() < deadline:
        time.sleep(0.5)
        enabled, active = query_state(name)
    log(f"当前状态 : is-enabled={enabled or '?'}  is-active={active or '?'}")
    if enabled == "masked":
        log("!! 仍是 masked：/etc/systemd/system/ 或 /run/systemd/system/ 下还有该 unit 的"
            "符号链接或 0 字节文件")
        log(f"   排查: ls -l /etc/systemd/system/{name} /run/systemd/system/{name}")
        log(f"   修复: sudo systemctl unmask {name}; sudo rm -f /etc/systemd/system/{name} "
            f"/run/systemd/system/{name}; sudo systemctl daemon-reload")
        return 1
    if active != "active":
        log("!! 服务不在 active：看日志找原因 —— ./install_service.py logs"
            "（常见：launch.command 路径/权限不对、串口被别的程序占用）")
        return 1
    log("服务已运行 ✓   查看日志: ./install_service.py logs")
    return 0


def cmd_print(cfg, user, python, watchdog_path, args):
    text = render_unit(cfg, user, python, watchdog_path)
    if args.dry_run:
        print(text, end="")
        return 0
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(text)
        os.sync()
        log(f"unit 已写入 {args.output}（已 sync 落盘）")
    else:
        print(text, end="")
    return 0


def cmd_install(cfg, user, python, watchdog_path, args):
    unit = render_unit(cfg, user, python, watchdog_path)
    dry = args.dry_run
    dest = unit_path(cfg.service_name, args.unit_dir)

    if args.output:
        # 只生成 unit 文件，跳过与安装环境相关的自检
        return cmd_print(cfg, user, python, watchdog_path, args)
    if not dry and not args.force:
        precheck(cfg, python, watchdog_path, dry)

    state = unit_state(dest)
    log(f"服务名   : {cfg.service_name}")
    log(f"运行用户 : {user}")
    log(f"解释器   : {python}")
    log(f"项目根   : {cfg.project_root}")
    log(f"看门狗   : {watchdog_path} --config {cfg.path}")
    log(f"unit 目标: {dest}（当前：{UNIT_STATE_DESC.get(state, state)}）")

    if state == "dir":
        raise SystemExit(f"[install_service] {dest} 是目录，请先人工删除后再安装")
    if dry:
        print(unit, end="")

    # ── 安装前：清理 mask；普通旧 unit 保留到新 unit 校验完成后再原子替换 ──
    # runtime mask 位于 /run/systemd/system，即使 /etc 下 unit 缺失也必须先 unmask
    run(sudo_prefix() + ["systemctl", "unmask", cfg.service_name], dry=dry)
    if state in ("symlink", "empty"):
        run(sudo_prefix() + ["rm", "-f", dest], dry=dry)
        rc = run(sudo_prefix() + ["systemctl", "daemon-reload"], dry=dry)
        require(rc, "清理旧 unit 后的 systemctl daemon-reload")

    if dry:
        tmp = "<临时 unit 文件>"
    else:
        tmp = None
        try:
            fd, tmp = tempfile.mkstemp(prefix="s2c-service-", suffix=".service")
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(unit)
                f.flush()
                os.fsync(f.fileno())     # 先把临时文件本身落盘
        except BaseException:
            if tmp is not None:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
            raise
    new_dest = dest + ".new"
    try:
        rc = run(sudo_prefix() + ["install", "-m", "0644", "-o", "root", "-g", "root",
                                  tmp, new_dest], dry=dry)
        require(rc, f"安装 unit 到 {new_dest}", "检查 sudo 权限/磁盘空间")
        if not dry:
            os.sync()            # ★ 关键：把新 unit 的数据块真正写到盘上，再断电也不会变 0 字节
            if not verify_installed_unit(new_dest, unit):
                raise SystemExit("[install_service] 已中止：unit 文件内容与生成结果不一致")
        rc = run(sudo_prefix() + ["mv", "-f", new_dest, dest], dry=dry)
        require(rc, f"原子替换 unit 到 {dest}")
        if not dry:
            os.sync()
        rc = run(sudo_prefix() + ["systemctl", "daemon-reload"], dry=dry)
        require(rc, "systemctl daemon-reload")
        if args.no_enable:
            log("已安装（按 --no-enable，未 enable/start）。需要时执行："
                f"sudo systemctl enable --now {cfg.service_name}")
            return 0
        rc = run(sudo_prefix() + ["systemctl", "enable", "--now", cfg.service_name], dry=dry)
        require(rc, f"systemctl enable --now {cfg.service_name}",
                "用 ./install_service.py status 看 unit 是否被 masked")
        # 已运行的 unit 不会因为 enable --now 而重启，这里重启一次让新 unit 生效
        rc = run(sudo_prefix() + ["systemctl", "try-restart", cfg.service_name], dry=dry)
        require(rc, f"systemctl try-restart {cfg.service_name}",
                "服务已安装但未能重启，请运行 ./install_service.py status 检查")
    finally:
        if not dry:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            run(sudo_prefix() + ["rm", "-f", new_dest], dry=False, check=False)

    if dry:
        return 0
    return report_state(cfg)


def cmd_uninstall(cfg, user, python, watchdog_path, args):
    dry = args.dry_run
    dest = unit_path(cfg.service_name, args.unit_dir)
    ok = True

    def step(cmd, what):
        nonlocal ok
        if run(cmd, dry=dry) != 0:
            log(f"[install_service] 卸载步骤失败: {what}")
            ok = False

    state = unit_state(dest)
    # 无条件解除 mask：runtime mask 位于 /run/systemd/system，dest 可能本来就不存在
    step(sudo_prefix() + ["systemctl", "unmask", cfg.service_name], "systemctl unmask")
    # unit 文件可能已被人工删除但 systemd 仍持有并运行它：先无条件尝试停止
    run(sudo_prefix() + ["systemctl", "stop", cfg.service_name], dry=dry, check=False)
    if state != "missing":
        step(sudo_prefix() + ["systemctl", "disable", "--now", cfg.service_name], "systemctl disable")
    step(sudo_prefix() + ["rm", "-f", dest], f"删除 {dest}")
    step(sudo_prefix() + ["systemctl", "daemon-reload"], "systemctl daemon-reload")
    if state != "missing":
        step(sudo_prefix() + ["systemctl", "reset-failed", cfg.service_name], "systemctl reset-failed")
    enabled, active = query_state(cfg.service_name)
    log(f"卸载后状态: is-enabled={enabled or '?'}  is-active={active or '?'}")
    if active == "active":
        log("[install_service] 卸载后服务仍处于 active，请用 systemctl status 检查")
        ok = False
    log(f"已卸载 {cfg.service_name}（项目文件与日志保留；日志文件: {cfg.log_file or 'journal'}）")
    return 0 if ok else 1


def cmd_status(cfg, user, python, watchdog_path, args):
    dest = unit_path(cfg.service_name, args.unit_dir)
    state = unit_state(dest)
    enabled, active = query_state(cfg.service_name)
    log(f"unit 文件: {dest} —— {UNIT_STATE_DESC.get(state, state)}")
    log(f"unit 状态: is-enabled={enabled or '?'}  is-active={active or '?'}")
    if state in ("empty", "symlink") or enabled == "masked":
        log("!! 这就是 'masked' 的原因：该路径是符号链接（mask）或 0 字节文件"
            "（systemd 对空 unit 同样按 masked 处理）。")
        log(f"   一键修复: sudo systemctl unmask {cfg.service_name}; sudo rm -f {dest}; "
            f"sudo systemctl daemon-reload   然后重新运行 ./install_service.py")
        if state == "empty":
            log("   提醒: 0 字节通常是「装完立刻断电」导致数据未落盘；装完记得 sync。")
    rc = run(["systemctl", "status", "--no-pager", "--full", cfg.service_name],
             dry=args.dry_run)
    if cfg.log_file and os.path.exists(cfg.log_file):
        log(f"看门狗日志文件: {cfg.log_file}")
    return rc


def cmd_logs(cfg, user, python, watchdog_path, args):
    cmd = ["journalctl", "-u", cfg.service_name, "-n",
           str(args.lines), "--no-pager", "-o", "short-iso"]
    return run(cmd, dry=args.dry_run)


def cmd_simple(action):
    def _run(cfg, user, python, watchdog_path, args):
        return run(sudo_prefix() + ["systemctl", action, cfg.service_name], dry=args.dry_run)
    return _run


def build_arg_parser():
    p = argparse.ArgumentParser(
        prog="install_service.py",
        description="把 auto_launch 看门狗安装为 systemd 系统服务（开机自启）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", nargs="?", default="install",
                   choices=["install", "uninstall", "status", "logs", "start", "stop",
                            "restart", "print"],
                   help="默认 install")
    p.add_argument("--config", default=os.path.join(HERE, DEFAULT_CONFIG_NAME),
                   help="auto_launch.conf 路径")
    p.add_argument("--user", default=None,
                   help="服务运行用户（默认 sudo 调用者/当前用户；需能访问串口）")
    p.add_argument("--python", default=None, help=f"解释器（默认 {DEFAULT_PYTHON}）")
    p.add_argument("--unit-dir", default=DEFAULT_UNIT_DIR,
                   help=f"unit 目录（默认 {DEFAULT_UNIT_DIR}；测试时可指向临时目录）")
    p.add_argument("--output", default=None,
                   help="只把 unit 写到该路径，不安装；与 --dry-run 同用时只打印")
    p.add_argument("--dry-run", action="store_true", help="只打印将要执行的命令，不做任何修改")
    p.add_argument("--no-enable", action="store_true", help="install 时只安装、不 enable/start")
    p.add_argument("--force", action="store_true", help="跳过安装前的 watchdog.py --check 自检")
    p.add_argument("--lines", type=int, default=200, help="logs 动作显示的行数")
    return p


def main(argv=None):
    args = build_arg_parser().parse_args(argv)

    try:
        cfg = Config(args.config)
    except ConfigError as e:
        raise SystemExit(f"[install_service] 配置错误: {e}")

    print_only = (args.action == "print") or (bool(args.output) and args.action == "install")
    user = detect_user(args.user)
    if not user:
        if print_only or args.dry_run or args.action in (
                "status", "logs", "uninstall", "stop", "start", "restart"):
            user = "root"   # 只读/卸载动作不需要真实服务用户
        else:
            raise SystemExit("[install_service] 当前以 root 运行且未指定运行用户，"
                             "请加 --user <用户名>（例如 --user rm1）")
    if not safe_user_exists(user):
        raise SystemExit(f"[install_service] 用户不存在: {user}")

    python = pick_python(args.python)
    if not os.path.isfile(python):
        raise SystemExit(f"[install_service] 解释器不存在: {python}")
    watchdog_path = os.path.join(HERE, "watchdog.py")

    # 按目标服务用户的权限位检查关键目录（sudo 下 os.access 用的是 root，无法代表服务用户）
    if not args.dry_run and not print_only and args.action == "install":
        access_checks = [
            ("项目根目录", cfg.project_root, os.R_OK | os.X_OK),
            ("锁/套接字目录", os.path.dirname(cfg.socket_path), os.W_OK | os.X_OK),
        ]
        if cfg.log_file:
            access_checks.append(("日志目录", os.path.dirname(cfg.log_file), os.W_OK | os.X_OK))
        for label, path, need in access_checks:
            if not os.path.isdir(path):
                log(f"警告: {label} {path} 不存在，服务可能启动失败")
            elif not user_can_access(user, path, need):
                log(f"警告: {label} {path} 对服务用户 {user} 不可访问，服务可能启动失败")
        serial_devices = []
        try:
            serial_devices = ["/dev/" + n for n in os.listdir("/dev") if n.startswith("ttyACM")]
        except OSError:
            pass
        if not serial_devices:
            log("警告: 未发现 /dev/ttyACM*，安装后服务可能没有串口可用")
        else:
            for dev in serial_devices:
                if not user_can_access(user, dev, os.R_OK | os.W_OK):
                    log(f"警告: 串口设备 {dev} 对服务用户 {user} 不可读写，"
                        "服务可能收不到电控数据")

    write_actions = {"install", "uninstall", "start", "stop", "restart"}
    if (args.action in write_actions and not is_root() and not args.dry_run
            and not print_only and not shutil.which("sudo")):
        raise SystemExit("[install_service] 需要 root 权限但没有 sudo，"
                         "请用 root 直接运行并在 install 时加 --user <用户名>")

    if args.action == "install":
        if not args.dry_run and not print_only:
            try:
                os.chmod(watchdog_path, 0o755)   # 允许直接 ./watchdog.py 手动运行
            except OSError as e:
                raise SystemExit(f"[install_service] 无法修改 {watchdog_path} 权限: {e}")
        return cmd_install(cfg, user, python, watchdog_path, args)
    if args.action == "print":
        return cmd_print(cfg, user, python, watchdog_path, args)
    if args.action == "uninstall":
        return cmd_uninstall(cfg, user, python, watchdog_path, args)
    if args.action == "status":
        return cmd_status(cfg, user, python, watchdog_path, args)
    if args.action == "logs":
        return cmd_logs(cfg, user, python, watchdog_path, args)
    return cmd_simple(args.action)(cfg, user, python, watchdog_path, args)


if __name__ == "__main__":
    sys.exit(main())
