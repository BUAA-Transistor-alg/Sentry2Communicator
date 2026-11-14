#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""auto_launch/watchdog.py — 串口通信服务端看门狗

职责（只做一件事：让主程序一直活着，且是「真的在干活」地活着）：
  1. 按 auto_launch.conf 的 [launch] 段执行运行主程序的命令行（命令行内容编码在配置文件里，
     本脚本不含任何针对具体可执行文件/参数的内容）；
  2. 进程级监控：子进程（及其后代）退出后按策略重启；
  3. 心跳级监控：主程序经 Unix 域套接字周期喂狗（见 include/common/Debug/WatchdogFeed.h），
     超时未喂狗即判定卡死（死锁 / 300Hz 发送循环停摆），杀掉整个进程组后重启；
  4. 退避重启：连续快速失败时指数退避（上限 restart_backoff_max_sec），避免崩溃风暴；
  5. 干净退出：SIGTERM/SIGINT → 向进程组发 SIGINT → 宽限后 SIGKILL，并清理本实例创建的套接字；
     SIGHUP → 重读配置文件并重启子进程（systemd 里对应 systemctl reload）。

用法：
  ./watchdog.py                     # 看门狗模式（systemd 服务用的就是这条）
  ./watchdog.py --run               # 不用看门狗，只按配置前台跑一次命令行
  ./watchdog.py --check             # 只检查配置/环境/可执行文件，不启动
  ./watchdog.py --self-test         # 用内置假程序跑通「心跳→卡死→杀掉→重启」全链路（约 20s）
  ./watchdog.py --print-command     # 打印将要在项目根目录执行的 shell 脚本
  常用覆盖：--config PATH  --command "..."  --no-heartbeat  --log-level debug
            --max-restarts N  --feed-timeout SEC  --connect-timeout SEC

注意：
  - 本脚本只用 Python 标准库（systemd 下用 /usr/bin/python3 即可，不依赖 venv）；
  - 同一时刻只允许一个看门狗实例（对 socket_path + ".lock" 加 flock，进程崩溃后自动释放）；
  - 退出码：0 = 正常停止（收到信号 / restart_policy=unexpected 下子进程正常退出）；
    非 0 = 启动/配置失败，或达到 max_restarts。
"""

import argparse
import configparser
import fcntl
import math
import os
import re
import select
import shlex
import signal
import socket
import stat
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from shutil import which as shutil_which

DEFAULT_CONFIG_NAME = "auto_launch.conf"

# ── 配置默认值（与 auto_launch.conf 中的注释保持一致）──
DEFAULTS = {
    "launch": {
        "command": "",
        "preamble": "",
        "shell": "/bin/bash",
        "project_root": "..",
        "restart_policy": "always",
    },
    "watchdog": {
        "heartbeat_enabled": "true",
        "socket_path": "/tmp/s2c_serial_comm_watchdog.sock",
        "feed_timeout_sec": "20",
        "connect_timeout_sec": "30",
        "shutdown_grace_sec": "5",
        "restart_delay_sec": "3",
        "restart_backoff_max_sec": "30",
        "healthy_run_sec": "30",
        "max_restarts": "0",
        "feed_period_ms": "500",
        "stall_timeout_ms": "10000",
        "boot_grace_ms": "10000",
        "log_file": "logs/watchdog.log",
        "log_max_bytes": str(5 * 1024 * 1024),
        "log_level": "info",
        "status_log_sec": "10",
    },
    "service": {
        "name": "s2c_serial_comm.service",
        "description": "Sentry2 Serial Communication Server (auto_launch watchdog)",
        "after": "network-online.target",
        "wants": "network-online.target",
        "environment": "PYTHONUNBUFFERED=1",
        "restart_sec": "3",
        "timeout_stop_sec": "20",
    },
}

LOG_LEVELS = {"debug": 10, "info": 20, "warn": 30, "warning": 30, "error": 40}
_UNSET = object()   # 区分“未传入”与显式 None
HEARTBEAT_RE = re.compile(r"^S2C-WD/1\s+FEED\s+seq=(\d+)\s+frames=(\d+)\s*$")

# shell 元字符：命令不含这些字符时用 exec 直接替换 shell（信号直达、退出码即命令退出码）
_SHELL_META = set("|&;<>()$`\\\"'*?[]{}~#\n")


class ConfigError(Exception):
    """配置文件错误（缺项/取值非法/路径不存在）"""


def log_stamp():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


class Logger:
    """同时写 stdout（systemd 下进 journal）与可选日志文件；文件超过上限时轮转为 .1"""

    def __init__(self, level="info", path=None, max_bytes=0):
        self.level = LOG_LEVELS.get(str(level).lower(), 20)
        self.path = path
        self.max_bytes = int(max_bytes or 0)
        self.fh = None
        self._open()

    def _open(self):
        if not self.path:
            return
        try:
            parent = os.path.dirname(self.path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            if self.max_bytes > 0 and os.path.exists(self.path) \
                    and os.path.getsize(self.path) >= self.max_bytes:
                os.replace(self.path, self.path + ".1")
            self.fh = open(self.path, "a", encoding="utf-8", buffering=1)
        except OSError as e:
            self.fh = None
            sys.stderr.write(f"[watchdog] 无法写日志文件 {self.path}: {e}（改为只输出到 stdout）\n")

    def close(self):
        if self.fh:
            try:
                self.fh.close()
            finally:
                self.fh = None

    def reconfigure(self, level=None, path=_UNSET, max_bytes=None):
        """重载配置时更新级别/文件（路径变化则重新打开；显式空路径切回 stdout）"""
        if level is not None:
            self.level = LOG_LEVELS.get(str(level).lower(), self.level)
        # 先更新阈值再打开新文件，_open 的初始化轮转使用本次配置值
        if max_bytes is not None:
            self.max_bytes = max_bytes
        if path is not _UNSET and path != self.path:
            self.close()
            self.path = path
            self._open()

    def _maybe_rotate(self):
        if not self.fh or self.max_bytes <= 0:
            return
        try:
            size = os.path.getsize(self.path)
        except OSError:
            # 日志文件被外部删除/改名：关闭旧句柄并按当前路径重新打开
            self.close()
            self._open()
            return
        if size <= 0 or size < self.max_bytes:
            return
        try:
            self.fh.close()
        except OSError:
            pass
        self.fh = None
        try:
            os.replace(self.path, self.path + ".1")
        except OSError:
            pass
        try:
            self.fh = open(self.path, "a", encoding="utf-8", buffering=1)
        except OSError as e:
            self.fh = None
            sys.stderr.write(f"[watchdog] 轮转后无法重新打开日志 {self.path}: {e}（改为只输出到 stdout）\n")

    def _write(self, tag, msg):
        line = f"{log_stamp()} [{tag}] {msg}"
        print(line, flush=True)
        if self.fh:
            self._maybe_rotate()
            if self.fh:
                try:
                    self.fh.write(line + "\n")
                except OSError:
                    pass

    def debug(self, msg, *a):
        if self.level <= 10:
            self._write("debug", msg % a if a else msg)

    def info(self, msg, *a):
        if self.level <= 20:
            self._write("info", msg % a if a else msg)

    def warn(self, msg, *a):
        if self.level <= 30:
            self._write("warn", msg % a if a else msg)

    def error(self, msg, *a):
        self._write("error", msg % a if a else msg)


class Config:
    """auto_launch.conf 的解析结果（含路径解析与合法性校验）"""

    def __init__(self, path, cli=None):
        self.path = os.path.abspath(path)
        if not os.path.isfile(self.path):
            raise ConfigError(f"配置文件不存在: {self.path}")
        self.base_dir = os.path.dirname(self.path)

        # 允许行内注释（" #..." / " ;..."，需前置空白）——配置文件里大量使用行内注释说明取值
        parser = configparser.ConfigParser(interpolation=None,
                                           inline_comment_prefixes=("#", ";"))
        # 保留 key 大小写不敏感（默认已是），并允许值里出现 '%'
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                parser.read_file(f)
        except (configparser.Error, OSError, UnicodeDecodeError) as e:
            raise ConfigError(f"配置文件解析失败: {e}")

        self._parser = parser
        cli = cli or {}

        def cli_or(key, default):
            value = cli.get(key)
            return default if value is None else value
        self.raw = {sec: dict(DEFAULTS[sec]) for sec in DEFAULTS}
        for sec in DEFAULTS:
            if parser.has_section(sec):
                for key, val in parser.items(sec):
                    self.raw[sec][key] = val.strip()
        # 未知键会让拼写错误静默回退默认值，直接拒绝
        for sec in parser.sections():
            if sec not in DEFAULTS:
                raise ConfigError(f"未知配置段: [{sec}]")
            for key in parser[sec]:
                if key not in DEFAULTS[sec]:
                    raise ConfigError(f"未知配置项: [{sec}] {key}")

        # ── [launch] ──
        self.project_root = os.path.abspath(
            os.path.join(self.base_dir, self.raw["launch"]["project_root"] or ".."))
        self.command = str(cli_or("command", self.raw["launch"]["command"])).strip()
        self.preamble = self.raw["launch"]["preamble"].strip()
        self.shell = self.raw["launch"]["shell"].strip() or "/bin/bash"
        self.restart_policy = self.raw["launch"]["restart_policy"].strip().lower()

        # ── [watchdog] ──
        w = self.raw["watchdog"]
        self.heartbeat_enabled = parse_bool(w["heartbeat_enabled"], "watchdog.heartbeat_enabled")
        if cli.get("no_heartbeat"):
            self.heartbeat_enabled = False
        elif cli.get("heartbeat"):
            self.heartbeat_enabled = True
        self.socket_path = str(cli_or("socket_path", w["socket_path"])).strip()
        self.feed_timeout_sec = parse_float(
            cli_or("feed_timeout", w["feed_timeout_sec"]), "watchdog.feed_timeout_sec")
        self.connect_timeout_sec = parse_float(
            cli_or("connect_timeout", w["connect_timeout_sec"]), "watchdog.connect_timeout_sec")
        self.shutdown_grace_sec = parse_float(w["shutdown_grace_sec"], "watchdog.shutdown_grace_sec")
        self.restart_delay_sec = parse_float(w["restart_delay_sec"], "watchdog.restart_delay_sec")
        self.restart_backoff_max_sec = parse_float(
            w["restart_backoff_max_sec"], "watchdog.restart_backoff_max_sec")
        self.healthy_run_sec = parse_float(w["healthy_run_sec"], "watchdog.healthy_run_sec")
        self.max_restarts = parse_int(
            cli_or("max_restarts", w["max_restarts"]), "watchdog.max_restarts")
        self.feed_period_ms = parse_int(
            cli_or("feed_period", w["feed_period_ms"]), "watchdog.feed_period_ms")
        self.stall_timeout_ms = parse_int(
            cli_or("stall_timeout", w["stall_timeout_ms"]), "watchdog.stall_timeout_ms")
        self.boot_grace_ms = parse_int(
            cli_or("boot_grace", w["boot_grace_ms"]), "watchdog.boot_grace_ms")
        self.log_level = str(cli_or("log_level", w["log_level"])).strip().lower()
        self.log_max_bytes = parse_int(w["log_max_bytes"], "watchdog.log_max_bytes")
        self.status_log_sec = parse_float(w["status_log_sec"], "watchdog.status_log_sec")
        if self.status_log_sec <= 0:
            self.status_log_sec = float("inf")   # <=0 表示不做周期性状态日志
        log_file = w["log_file"].strip()
        if cli.get("no_log"):
            log_file = ""
        elif cli.get("log_file"):
            log_file = cli["log_file"]
        self.log_file = os.path.abspath(os.path.join(self.project_root, log_file)) if log_file else ""

        # ── [service] ──
        s = self.raw["service"]
        self.service_name = s["name"].strip()
        self.service_description = s["description"].strip()
        self.service_after = split_lines(s["after"])
        self.service_wants = split_lines(s["wants"])
        self.service_environment = split_lines(s["environment"])
        self.service_restart_sec = parse_int(s["restart_sec"], "service.restart_sec")
        self.service_timeout_stop_sec = parse_int(s["timeout_stop_sec"], "service.timeout_stop_sec")

        self.validate()

    # ── 校验 ──
    def validate(self):
        if not self.command:
            raise ConfigError("launch.command 为空：请在 auto_launch.conf 里写好运行主程序的命令行")
        if not os.path.isdir(self.project_root):
            raise ConfigError(f"launch.project_root 不是目录: {self.project_root}")
        if not os.path.isfile(self.shell) or not os.access(self.shell, os.X_OK):
            raise ConfigError(f"launch.shell 不存在或不可执行: {self.shell}")
        if self.restart_policy not in ("always", "unexpected"):
            raise ConfigError("launch.restart_policy 只能是 always 或 unexpected，"
                              f"当前为 {self.restart_policy!r}")
        if self.log_level not in LOG_LEVELS:
            raise ConfigError(f"watchdog.log_level 非法: {self.log_level!r}")
        # 锁文件路径由 socket_path 派生，无论是否启用心跳都必须合法
        if not self.socket_path.startswith("/"):
            raise ConfigError(f"watchdog.socket_path 必须是绝对路径: {self.socket_path!r}")
        if len(self.socket_path.encode()) > 107:
            raise ConfigError(f"watchdog.socket_path 过长（Unix 域套接字上限 107 字节）: "
                              f"{self.socket_path}")
        if self.heartbeat_enabled:
            if self.feed_timeout_sec <= 0:
                raise ConfigError("watchdog.feed_timeout_sec 必须 > 0")
            if self.connect_timeout_sec <= 0:
                raise ConfigError("watchdog.connect_timeout_sec 必须 > 0")
            if self.feed_period_ms <= 0:
                raise ConfigError("watchdog.feed_period_ms 必须 > 0")
            if self.feed_period_ms < 50:
                raise ConfigError("watchdog.feed_period_ms 不能小于 50（WatchdogFeed 的最小上报周期）")
            timeout_ms = self.feed_timeout_sec * 1000.0
            if self.feed_period_ms >= timeout_ms:
                raise ConfigError("watchdog.feed_period_ms 必须小于 feed_timeout_sec 的毫秒值")
            if self.feed_period_ms * 2 >= timeout_ms:
                raise ConfigError("watchdog.feed_period_ms 必须小于 feed_timeout_sec 毫秒值的一半，"
                                  "为调度抖动留出余量")
            if self.stall_timeout_ms > 0 and self.stall_timeout_ms >= timeout_ms:
                raise ConfigError("watchdog.stall_timeout_ms 必须小于 feed_timeout_sec 的毫秒值")
            if self.connect_timeout_sec * 1000.0 < self.boot_grace_ms + timeout_ms:
                raise ConfigError("watchdog.connect_timeout_sec 必须覆盖 boot_grace_ms 与 feed_timeout_sec")
        if self.shutdown_grace_sec < 0:
            raise ConfigError("watchdog.shutdown_grace_sec 不能为负")
        if self.restart_delay_sec <= 0:
            raise ConfigError("watchdog.restart_delay_sec 必须 > 0")
        if self.restart_backoff_max_sec <= 0:
            raise ConfigError("watchdog.restart_backoff_max_sec 必须 > 0")
        if self.healthy_run_sec < 0:
            raise ConfigError("watchdog.healthy_run_sec 不能为负")
        if self.log_max_bytes < 0:
            raise ConfigError("watchdog.log_max_bytes 不能为负")
        if self.max_restarts < 0:
            raise ConfigError(f"watchdog.max_restarts 不能为负: {self.max_restarts}")
        if (not self.service_name or "/" in self.service_name or "\\" in self.service_name
                or ".." in self.service_name or self.service_name != self.service_name.strip()
                or any(ch.isspace() for ch in self.service_name)
                or not self.service_name.endswith(".service")):
            raise ConfigError("service.name 必须是合法的 .service 单元名（不能包含路径分隔符或空白）")

    # ── 派生量 ──
    @property
    def lock_path(self):
        return self.socket_path + ".lock"

    def child_env(self):
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        for key in ("WATCHDOG_SOCKET_PATH", "WATCHDOG_FEED_PERIOD_MS",
                    "WATCHDOG_STALL_TIMEOUT_MS", "WATCHDOG_BOOT_GRACE_MS"):
            env.pop(key, None)
        if self.heartbeat_enabled:
            env["WATCHDOG_SOCKET_PATH"] = self.socket_path
            env["WATCHDOG_FEED_PERIOD_MS"] = str(self.feed_period_ms)
            env["WATCHDOG_STALL_TIMEOUT_MS"] = str(self.stall_timeout_ms)
            env["WATCHDOG_BOOT_GRACE_MS"] = str(self.boot_grace_ms)
        return env

    def shell_script(self):
        """把 preamble + command 组装成一段 shell 脚本（命令行内容全部来自配置文件）

        命令是「简单可执行命令」时用 exec 直接替换 shell（信号直达、退出码即命令退出码）；
        否则原样执行（复合语句 / shell 内建命令如 cd、export 不能 exec，照样在同一个
        进程组里，看门狗 killpg 一样能整组清干净）。
        """
        lines = ["set -e"]
        if self.preamble:
            lines.extend(self.preamble.splitlines())
        lines.append(("exec " if use_exec(self.command, self.project_root) else "") + self.command)
        return "\n".join(lines) + "\n"

    def summary(self):
        return "\n".join([
            f"配置文件   : {self.path}",
            f"项目根目录 : {self.project_root}",
            f"启动命令   : {self.command}",
            f"shell      : {self.shell}"
            + (f"（含 {len(self.preamble.splitlines())} 行 preamble）" if self.preamble else ""),
            f"重启策略   : {self.restart_policy}",
            "喂狗通道   : "
            + (f"开启 → {self.socket_path}（超时 {self.feed_timeout_sec:g}s，"
               f"首次连接 {self.connect_timeout_sec:g}s，"
               f"帧门控 {self.stall_timeout_ms}ms，启动宽限 {self.boot_grace_ms}ms）"
               if self.heartbeat_enabled else "关闭（只按进程存活判定）"),
            f"退避重启   : 基础 {self.restart_delay_sec:g}s，上限 {self.restart_backoff_max_sec:g}s，"
            f"稳定判定 {self.healthy_run_sec:g}s"
            + ("，不限次数" if self.max_restarts == 0 else f"，最多 {self.max_restarts} 次"),
            f"日志       : {self.log_file or 'stdout（journal）'}"
            f"（级别 {self.log_level}）",
        ])


def parse_bool(text, what):
    val = str(text).strip().lower()
    if val in ("1", "true", "yes", "on", "y"):
        return True
    if val in ("0", "false", "no", "off", "n", ""):
        return False
    raise ConfigError(f"{what} 需要 true/false，当前为 {text!r}")


def parse_int(text, what):
    try:
        return int(str(text).strip())
    except ValueError:
        raise ConfigError(f"{what} 需要整数，当前为 {text!r}")


def parse_float(text, what):
    try:
        value = float(str(text).strip())
    except ValueError:
        raise ConfigError(f"{what} 需要数值，当前为 {text!r}")
    if not math.isfinite(value):
        raise ConfigError(f"{what} 必须是有限数值，当前为 {text!r}")
    return value


def split_lines(text):
    """多行配置值 → 非空行列表（去掉行内注释）"""
    out = []
    for line in str(text).splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith(";"):
            continue
        out.append(line)
    return out


def is_simple_command(cmd):
    return not any(ch in _SHELL_META for ch in cmd)


def use_exec(cmd, project_root):
    """能否用 exec 替换 shell：命令无 shell 元字符且首词确实是个可执行文件

    为什么要判断：`exec cd x` / `exec export A=1` 这类 shell 内建命令会直接失败，
    而 `cd x && ./bin/y` 这种复合语句 exec 语义也不对——这些情况原样交给 shell 执行。
    """
    if not is_simple_command(cmd):
        return False
    parts = cmd.split()
    if not parts:
        return False
    first = parts[0]
    if "/" in first:
        path = first if os.path.isabs(first) else os.path.join(project_root, first)
        return os.path.isfile(path) and os.access(path, os.X_OK)
    return shutil_which(first) is not None


# 以这些词开头的命令交给 shell 解释，首词不是可执行文件
_SHELL_KEYWORDS = {"exec", "export", "source", ".", "cd", "set", "unset", "alias",
                   "env", "command", "builtin", "ulimit", "umask", "trap", "eval"}


def check_command(cfg):
    """检查配置命令能否执行；返回 (是否通过, 检查项名称, 细节说明)"""
    parts = cfg.command.split()
    first = parts[0] if parts else ""
    if not is_simple_command(cfg.command) or not first:
        return True, "命令为复合 shell 语句（跳过首词检查）", cfg.command[:48]
    if "=" in first or first in _SHELL_KEYWORDS:
        return True, "命令以 shell 赋值/内建命令开头（跳过首词检查）", first
    path = os.path.join(cfg.project_root, first) if "/" in first else None
    found = bool((path and os.path.isfile(path) and os.access(path, os.X_OK)) or
                 shutil_which(first))
    detail = path if path else "在 PATH 中查找"
    return found, f"命令首词（{first}）", detail if found else f"不可执行: {detail}"


class Watchdog:
    """看门狗主体：起子进程 → 等首次心跳 → 监控（进程存活 + 心跳）→ 退出/超时则重启"""

    def __init__(self, cfg, log, args):
        self.cfg = cfg
        self.log = log
        self.args = args
        self.child = None            # subprocess.Popen
        self.sock = None             # 心跳用的 Unix 域数据报套接字
        self.socket_created = False  # 套接字是否由本实例成功 bind（决定退出时是否 unlink）
        self.socket_identity = None  # bind 时套接字文件的 (st_dev, st_ino)
        self.socket_error_logged = False
        self.socket_unavailable_since = None
        self.socket_path_blocked = False
        self.socket_missing_since = None
        self.socket_rebuilt = False
        self.lock_fd = None
        self.stop_requested = False
        self.reload_requested = False
        self.started_at = 0.0        # 本实例启动时刻（用于日志）
        self.spawn_time = 0.0        # 当前子进程启动时刻
        self.connect_epoch = 0.0     # 首次心跳窗口起点（套接字恢复时重置）
        self.heartbeat_outage_sec = 0.0  # 本地套接字故障累计时长（健康判定时扣除）
        self.last_feed = None        # 最近一次收到心跳的时刻
        self.heartbeat_stat = ""     # 最近一次心跳内容（日志用）
        self.restarts = 0            # 已重启次数
        self.failures = 0            # 连续快速失败次数（退避用）

    # ────────────────────────── 信号 ──────────────────────────
    def install_signal_handlers(self):
        signal.signal(signal.SIGTERM, self._on_stop_signal)
        signal.signal(signal.SIGINT, self._on_stop_signal)
        signal.signal(signal.SIGHUP, self._on_reload_signal)

    def _on_stop_signal(self, signum, _frame):
        # 只置位，实际收尾在主循环里做（信号处理器里不做复杂操作）
        self.stop_requested = True

    def _on_reload_signal(self, signum, _frame):
        self.reload_requested = True

    # ────────────────────────── 资源 ──────────────────────────
    def _acquire_lock_at(self, path):
        """对指定路径加 flock 独占锁，返回锁文件描述符；失败抛 ConfigError"""
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
        except OSError as e:
            raise ConfigError(f"无法创建锁文件目录 {os.path.dirname(path)}: {e}")
        try:
            fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
        except OSError as e:
            # 典型场景：systemd 服务以 rm1 运行过，锁文件属主是 rm1 且权限 0644，
            # 现在以另一个用户手动跑就会 EACCES —— 这里给出可操作的提示
            raise ConfigError(
                f"无法打开锁文件 {path}: {e}。若已有以其它用户运行的看门狗/服务，请先停掉它"
                "（systemctl status <服务名>），或改用其它 socket_path（--config 指向不同配置）")
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            raise ConfigError(
                f"已有看门狗实例在运行（锁文件 {path} 被占用）。"
                "如需确认：ps -ef | grep watchdog.py；如为残留：删除该锁文件")
        return fd

    def _release_lock_fd(self, fd):
        if fd is None:
            return
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        except OSError:
            pass
        try:
            os.close(fd)
        except OSError:
            pass

    def acquire_lock(self):
        """flock 独占锁：防止两个看门狗同时拉起两套程序"""
        self.lock_fd = self._acquire_lock_at(self.cfg.lock_path)

    def _close_socket(self, remove_file=True):
        """关闭本实例的套接字；仅在路径仍指向本实例 bind 的 inode 时删除文件"""
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        if remove_file and self.socket_created and self.socket_identity is not None:
            try:
                st = os.stat(self.cfg.socket_path)
                if (st.st_dev, st.st_ino) == self.socket_identity:
                    os.unlink(self.cfg.socket_path)
            except OSError as e:
                self.log.debug("删除套接字失败 %s: %s", self.cfg.socket_path, e)
        self.socket_created = False
        self.socket_identity = None

    def open_socket(self):
        """建立心跳监听套接字（AF_UNIX + SOCK_DGRAM：无连接状态，主程序/看门狗各自重启都不影响对方）"""
        if not self.cfg.heartbeat_enabled:
            return
        path = self.cfg.socket_path
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            if os.path.exists(path):
                if not stat.S_ISSOCK(os.stat(path).st_mode):
                    raise ConfigError(f"{path} 已存在且不是套接字文件，拒绝覆盖")
                os.unlink(path)
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            try:
                sock.bind(path)
            except OSError as e:
                sock.close()
                raise ConfigError(f"绑定心跳套接字失败 {path}: {e}")
            # 先登记所有权：后续 chmod/setblocking 失败时也能清理本实例的套接字
            self.sock = sock
            self.socket_created = True
            st = os.stat(path)
            self.socket_identity = (st.st_dev, st.st_ino)   # 先登记所有权，chmod/setblocking 失败也能清理
            self.socket_path_blocked = False
            self.socket_missing_since = None
            self.socket_unavailable_since = None
            os.chmod(path, 0o666)   # 允许同机其它用户（如手动运行的调试进程）喂狗
            sock.setblocking(False)  # 非阻塞接收，drain 时不会因为持续到包而不返回
        except OSError as e:
            self._close_socket()
            raise ConfigError(f"建立心跳套接字失败 {path}: {e}")
        self.log.info("心跳套接字已监听: %s", path)

    def release(self):
        """关闭套接字并释放锁（幂等）；只删除本实例成功创建的套接字文件"""
        self._close_socket()
        self._release_lock_fd(self.lock_fd)
        self.lock_fd = None

    # ────────────────────────── 子进程 ──────────────────────────
    def spawn(self):
        script = self.cfg.shell_script()
        self.log.info("启动: %s -c %s", self.cfg.shell, shlex.quote(script))
        self.log.debug("工作目录: %s", self.cfg.project_root)
        # 启动前清掉旧进程残留的心跳报文，避免新进程还没喂狗就被判定为已喂狗
        self.spawn_time = time.monotonic()
        self.connect_epoch = self.spawn_time
        self.heartbeat_outage_sec = 0.0
        self.last_feed = None
        self.socket_missing_since = None
        self.socket_unavailable_since = None
        self.heartbeat_stat = ""
        self._discard_heartbeats()
        try:
            self.child = subprocess.Popen(
                [self.cfg.shell, "-c", script],
                cwd=self.cfg.project_root,
                env=self.cfg.child_env(),
                preexec_fn=os.setsid,   # 自成进程组：重启/退出时可整组 SIGINT→SIGKILL
            )
        except OSError as e:
            self.child = None
            self.log.error("启动子进程失败: %s", e)
            raise   # 交给 main 以退出码 1 结束，systemd 按 Restart=always 重试
        self.log.info("子进程已启动: pid=%d (pgid=%d)", self.child.pid, self.child.pid)

    def kill_child(self, reason, signum=signal.SIGINT):
        """整组停止子进程：SIGINT → 宽限 → SIGKILL。子进程已退出时也会清掉残留后代。"""
        if self.child is None:
            return
        pid = self.child.pid
        pgid = pid
        if self.child.poll() is None:
            self.log.warn("停止进程组 %d（%s）：先发 %s", pgid, reason, signal.Signals(signum).name)
            self._signal_group(pgid, signum)
            try:
                self.child.wait(timeout=self.cfg.shutdown_grace_sec)
                self.log.info("子进程 %d 已退出（退出码 %s）", pid, self.child.returncode)
                self._signal_group(pgid, signal.SIGKILL)   # 兜底清理未退出的后代
                self.child = None
                return
            except subprocess.TimeoutExpired:
                self.log.warn("宽限 %gs 内未退出，升级为 SIGKILL", self.cfg.shutdown_grace_sec)
        self._signal_group(pgid, signal.SIGKILL)
        try:
            self.child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.log.error("子进程 %d 无法回收（僵尸残留）", pid)
            self.child = None
            raise ConfigError("子进程无法回收，停止看门狗避免双实例")
        else:
            self.log.info("进程组 %d 已强制终止", pgid)
        self.child = None

    def _signal_group(self, pgid, signum):
        try:
            os.killpg(pgid, signum)
        except ProcessLookupError:
            pass
        except OSError as e:
            self.log.debug("killpg(%d, %d) 失败: %s", pgid, signum, e)

    # ────────────────────────── 心跳 ──────────────────────────
    def drain_heartbeats(self):
        """非阻塞收干套接字缓冲区；只把合法心跳报文计入存活，返回是否收到合法心跳"""
        got = False
        if not self.sock:
            return got
        # 单次最多清空 256 条，持续灌包时也能返回主循环处理退出/重载
        for _ in range(256):
            try:
                readable, _, _ = select.select([self.sock], [], [], 0)
            except OSError as e:
                self.log.debug("select 失败: %s", e)
                break
            if not readable:
                break
            try:
                data, _addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as e:
                self.log.debug("recvfrom 失败: %s", e)
                break
            if not data:
                continue
            text = data.decode("utf-8", "replace").strip()
            m = HEARTBEAT_RE.match(text)
            if m is None:
                self.log.debug("忽略非心跳报文: %r", text[:64])
                continue
            self.last_feed = time.monotonic()
            self.heartbeat_stat = f"frames={m.group(2)}"
            got = True
        return got

    def _ensure_socket(self):
        """心跳套接字文件被 /tmp 清理或替换后重新绑定"""
        if not self.cfg.heartbeat_enabled:
            return
        path = self.cfg.socket_path
        if self.socket_path_blocked:
            if os.path.lexists(path):
                return   # 路径仍被其他对象占用，不再删除重绑
            self.socket_path_blocked = False
        if self.sock is not None:
            try:
                st = os.stat(path)
            except OSError:
                self.log.warn("心跳套接字文件丢失，重新绑定: %s", path)
                if self.socket_missing_since is None:
                    self.socket_missing_since = time.monotonic()
                self.socket_rebuilt = True
                self._close_socket(remove_file=False)
            else:
                if self.socket_identity == (st.st_dev, st.st_ino):
                    return
                # 路径已被其他对象占用：只关闭本实例监听，不删除不属于自己的文件
                self.log.error("心跳套接字路径已被其他对象占用，停止重新绑定: %s", path)
                self.socket_path_blocked = True
                self._close_socket(remove_file=False)
                return
        missing_since = self.socket_missing_since
        try:
            self.open_socket()
            self.socket_error_logged = False
            if missing_since is not None:
                self.socket_unavailable_since = missing_since
        except ConfigError as e:
            if not self.socket_error_logged:
                self.log.error("重新绑定心跳套接字失败: %s", e)
                self.socket_error_logged = True

    def _discard_heartbeats(self):
        """丢弃套接字里残留的报文，不刷新 last_feed"""
        if not self.sock:
            return
        for _ in range(256):
            try:
                readable, _, _ = select.select([self.sock], [], [], 0)
            except OSError:
                break
            if not readable:
                break
            try:
                self.sock.recvfrom(4096)
            except (BlockingIOError, OSError):
                break

    # ────────────────────────── 监控 ──────────────────────────
    def monitor(self):
        """监控当前子进程，返回结束原因：
        ('stopped', None)      —— 收到停止信号
        ('reload', None)       —— 收到 SIGHUP
        ('exited', returncode) —— 子进程退出
        ('no_heartbeat', None) —— 起不来：connect_timeout 内没有首次喂狗
        ('feed_timeout', None) —— 卡死：feed_timeout 内没有喂狗
        """
        next_status_log = time.monotonic() + self.cfg.status_log_sec
        while True:
            if self.stop_requested:
                return ("stopped", None)
            if self.reload_requested:
                return ("reload", None)

            rc = self.child.poll()
            if rc is not None:
                return ("exited", rc)

            if self.cfg.heartbeat_enabled:
                had_socket = self.sock is not None
                self.socket_rebuilt = False
                self._ensure_socket()
                if self.sock is not None and (not had_socket or self.socket_rebuilt):
                    now2 = time.monotonic()
                    outage = 0.0
                    if self.socket_unavailable_since is not None:
                        outage = max(0.0, now2 - self.socket_unavailable_since)
                    if self.last_feed is not None and outage > 0.0:
                        # 扣除本地套接字故障占用的时间，并限制不超过当前子进程已运行时长
                        outage = min(outage, max(0.0, now2 - self.spawn_time))
                        self.last_feed = min(self.last_feed + outage, now2)
                        self.heartbeat_outage_sec += outage
                    self.connect_epoch = now2
                    self.socket_unavailable_since = None
                    self.log.info("心跳套接字恢复，重开首次心跳计时")
                self.drain_heartbeats()
                now = time.monotonic()
                if self.sock is None:
                    # 本地套接字不可用：短暂故障先只做进程级监控，超过超时阈值按配置错误退出
                    if self.socket_unavailable_since is None:
                        self.socket_unavailable_since = time.monotonic()
                    limit = max(self.cfg.connect_timeout_sec, self.cfg.feed_timeout_sec)
                    if now - self.socket_unavailable_since > limit:
                        raise ConfigError("心跳套接字持续不可用，看门狗无法判断子进程存活")
                    time.sleep(0.1)
                    continue
                self.socket_unavailable_since = None
                if self.last_feed is None:
                    if now - self.connect_epoch > self.cfg.connect_timeout_sec:
                        return ("no_heartbeat", None)
                elif now - self.last_feed > self.cfg.feed_timeout_sec:
                    self.log.warn("距上次喂狗 %.1fs（> %.1fs）：判定主程序卡死",
                                  now - self.last_feed, self.cfg.feed_timeout_sec)
                    return ("feed_timeout", None)

            now = time.monotonic()
            if now >= next_status_log:
                next_status_log = now + self.cfg.status_log_sec
                self.log_status(now)

            time.sleep(0.1)

    def log_status(self, now):
        run = now - self.spawn_time
        if self.cfg.heartbeat_enabled:
            feed = "尚未收到" if self.last_feed is None else f"{now - self.last_feed:.1f}s 前"
            self.log.info("运行中: pid=%s 已运行 %.0fs，最近心跳 %s %s",
                          self.child.pid if self.child else "-", run, feed, self.heartbeat_stat)
        else:
            self.log.info("运行中: pid=%s 已运行 %.0fs（未启用心跳）",
                          self.child.pid if self.child else "-", run)

    # ────────────────────────── 主循环 ──────────────────────────
    def run(self):
        self.started_at = time.monotonic()
        self.acquire_lock()
        self.open_socket()

        while not self.stop_requested:
            if self.reload_requested:
                self.reload_requested = False
                if self.reload_config() and self.child is not None:
                    self.kill_child("配置重载")

            if self.stop_requested:
                break
            # 重载失败时原 child 仍然存活，不能再次 spawn，否则会出现两个进程
            if self.child is None:
                self.spawn()
            reason, rc = self.monitor()

            if reason == "stopped":
                self.kill_child("收到停止信号")
                break

            if reason == "reload":
                self.reload_requested = False
                # 先校验并切换配置，成功后才停掉正在运行的子进程；
                # 配置无效时子进程继续按旧配置运行
                if self.reload_config():
                    self.kill_child("配置重载")
                continue

            # ── 子进程结束：先整组清干净，再决定是否重启 ──
            if reason == "exited":
                self.log.warn("子进程退出: 退出码 %s", format_returncode(rc))
                self.kill_child("子进程已退出，清理残留后代")
            else:
                self.kill_child("未喂狗（%s）" % ("启动超时" if reason == "no_heartbeat" else "心跳超时"))

            ran = time.monotonic() - self.spawn_time
            healthy = ran >= self.cfg.healthy_run_sec
            if reason in ("feed_timeout", "no_heartbeat"):
                # 心跳类失败按最后一次有效心跳之前的存活时长判定，避免检测时延把快速失败算成健康
                healthy = (self.last_feed is not None and
                           self.last_feed - self.spawn_time - self.heartbeat_outage_sec
                           >= self.cfg.healthy_run_sec)
            if healthy:
                self.failures = 0

            if self._should_stop_after(reason, rc):
                break

            if self.cfg.max_restarts and self.restarts >= self.cfg.max_restarts:
                self.log.error("已达 max_restarts=%d，停止重启", self.cfg.max_restarts)
                return 2

            delay = min(self.cfg.restart_delay_sec * (2 ** min(self.failures, 20)),
                        self.cfg.restart_backoff_max_sec)
            self.failures += 1
            self.restarts += 1
            self.log.info("第 %d 次重启，%.1fs 后重试", self.restarts, delay)
            if self._sleep_interruptible(delay):
                self.kill_child("收到停止信号")
                break

        # 统一清理：重载失败后收到停止信号等路径可能仍有存活的子进程
        if self.child is not None:
            self.kill_child("看门狗退出")

        self.log.info("看门狗退出（累计重启 %d 次，运行 %.0fs）",
                      self.restarts, time.monotonic() - self.started_at)
        return 0

    def _should_stop_after(self, reason, rc):
        """restart_policy=unexpected 时，子进程正常退出（退出码 0）不再重启"""
        if self.cfg.restart_policy != "unexpected":
            return False
        if reason == "exited" and rc == 0:
            self.log.info("restart_policy=unexpected 且子进程正常退出，不再重启")
            return True
        return False

    def _sleep_interruptible(self, seconds):
        """可被停止信号打断的睡眠；返回 True 表示期间收到停止请求"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.stop_requested:
                return True
            if self.reload_requested:
                return False   # 提前结束退避，让主循环处理配置重载
            time.sleep(min(0.1, max(0.0, end - time.monotonic())))
        return False

    def reload_config(self):
        """重读配置文件（SIGHUP / systemctl reload）：套接字与锁文件随 socket_path 一起迁移"""
        self.log.info("重载配置: %s", self.cfg.path)
        try:
            new_cfg = Config(self.cfg.path, self._cli_overrides())
            new_cfg.validate()
        except ConfigError as e:
            self.log.error("配置重载失败，继续使用旧配置: %s", e)
            return False
        cmd_ok, cmd_name, cmd_detail = check_command(new_cfg)
        if not cmd_ok:
            self.log.error("重载后的命令不可执行（%s: %s），继续使用旧配置",
                           new_cfg.command, cmd_detail)
            return False
        if new_cfg.restart_policy != self.cfg.restart_policy:
            self.log.error("restart_policy 变化需要重新安装 unit 才生效，继续使用旧配置")
            return False
        old_cfg = self.cfg
        socket_changed = ((new_cfg.heartbeat_enabled, new_cfg.socket_path) !=
                          (old_cfg.heartbeat_enabled, old_cfg.socket_path))
        old_log = (old_cfg.log_file, old_cfg.log_max_bytes)
        lock_moved = new_cfg.lock_path != old_cfg.lock_path

        # 先拿到新路径的锁，成功后才切换套接字；失败时保留旧锁与旧监听
        new_lock_fd = None
        if lock_moved:
            try:
                new_lock_fd = self._acquire_lock_at(new_cfg.lock_path)
            except ConfigError as e:
                self.log.error("新路径锁文件获取失败，继续使用旧配置: %s", e)
                return False

        if socket_changed:
            self._close_socket()   # 按旧 cfg 关闭并删除旧套接字
            self.cfg = new_cfg
            try:
                self.open_socket()
            except ConfigError as e:
                # 新路径监听失败：清掉新路径残留，恢复旧配置与旧监听
                self.log.error("新套接字监听失败，恢复旧配置: %s", e)
                self._close_socket()
                self.cfg = old_cfg
                try:
                    self.open_socket()
                except ConfigError as e2:
                    self.log.error("恢复旧套接字也失败: %s", e2)
                    self._release_lock_fd(new_lock_fd)
                    self.kill_child("无法恢复心跳套接字")
                    raise ConfigError(f"无法恢复心跳套接字: {e2}")
                self._release_lock_fd(new_lock_fd)
                return False
        else:
            self.cfg = new_cfg

        if new_lock_fd is not None:
            self._release_lock_fd(self.lock_fd)
            self.lock_fd = new_lock_fd

        if (new_cfg.log_file, new_cfg.log_max_bytes) != old_log:
            self.log.reconfigure(path=new_cfg.log_file, max_bytes=new_cfg.log_max_bytes)
        self.log.reconfigure(level=new_cfg.log_level)
        if old_cfg.raw["service"] != new_cfg.raw["service"]:
            self.log.warn("[service] 段变化需要重新执行 install_service.py 才会生效")
        self.log.info("配置已重载")
        return True

    def _cli_overrides(self):
        return {
            "command": self.args.command,
            "no_heartbeat": self.args.no_heartbeat,
            "heartbeat": self.args.heartbeat,
            "feed_timeout": self.args.feed_timeout,
            "connect_timeout": self.args.connect_timeout,
            "max_restarts": self.args.max_restarts,
            "log_file": self.args.log_file,
            "no_log": self.args.no_log,
            "log_level": self.args.log_level,
            "socket_path": self.args.socket_path,
            "feed_period": self.args.feed_period,
            "stall_timeout": self.args.stall_timeout,
            "boot_grace": self.args.boot_grace,
        }


def format_returncode(rc):
    if rc is None:
        return "unknown"
    if rc < 0:
        try:
            return f"{rc} ({signal.Signals(-rc).name})"
        except ValueError:
            return str(rc)
    return str(rc)


def run_foreground(cfg, log, args):
    """--run：不看门狗，只在项目根目录前台执行配置里的命令行"""
    script = cfg.shell_script()
    if args.print_command:
        print(f"# 项目根目录: {cfg.project_root}")
        print(f"# shell: {cfg.shell}")
        print(script, end="")
        return 0
    log.info("前台运行（无看门狗）: %s -c %s", cfg.shell, shlex.quote(script))
    log.debug("工作目录: %s", cfg.project_root)
    # --run 不监听心跳套接字：去掉心跳环境变量，避免子进程向配置路径投递心跳
    env = cfg.child_env()
    for key in ("WATCHDOG_SOCKET_PATH", "WATCHDOG_FEED_PERIOD_MS",
                "WATCHDOG_STALL_TIMEOUT_MS", "WATCHDOG_BOOT_GRACE_MS"):
        env.pop(key, None)
    stop_state = {"received": False}
    proc = None

    def _fwd(signum, _frame):
        # 前台模式：Ctrl+C / kill 转发给整个进程组，宽限后升级为 SIGKILL
        stop_state["received"] = True
        if proc is not None:
            try:
                os.killpg(proc.pid, signum)
            except OSError:
                pass

    signal.signal(signal.SIGTERM, _fwd)
    signal.signal(signal.SIGINT, _fwd)
    signal.signal(signal.SIGHUP, _fwd)   # 终端断开/RUN 模式重载统一按停止处理，避免子进程失去管理
    proc = subprocess.Popen([cfg.shell, "-c", script], cwd=cfg.project_root,
                            env=env, preexec_fn=os.setsid)
    if stop_state["received"]:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except OSError:
            pass
    stop_deadline = None
    kill_sent = False
    while True:
        rc = proc.poll()
        if rc is not None:
            break
        if stop_state["received"]:
            if stop_deadline is None:
                stop_deadline = time.monotonic() + cfg.shutdown_grace_sec
            elif not kill_sent and time.monotonic() >= stop_deadline:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except OSError:
                    pass
                kill_sent = True
        time.sleep(0.1)
    rc = proc.wait()
    # 子进程已退出但可能还有忽略信号的后代，整组 SIGKILL 清理；无后代时 ESRCH 被忽略
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except OSError:
        pass
    log.info("运行结束: 退出码 %s", format_returncode(rc))
    return 0 if rc == 0 else 1


def check(cfg, log, args):
    """--check：不启动程序，只做静态检查（配置文件/路径/命令行/日志目录）"""
    ok = True
    print("配置检查:")
    print(indent(cfg.summary()))

    print("\n检查项:")
    checks = []

    shell_ok = os.path.isfile(cfg.shell) and os.access(cfg.shell, os.X_OK)
    checks.append(("shell 可执行", shell_ok, cfg.shell))

    cmd_ok, cmd_name, cmd_detail = check_command(cfg)
    checks.append((cmd_name, cmd_ok, cmd_detail))

    if cfg.heartbeat_enabled and os.path.lexists(cfg.socket_path):
        try:
            is_sock = stat.S_ISSOCK(os.stat(cfg.socket_path).st_mode)   # 与 open_socket 一样跟随符号链接
        except OSError:
            is_sock = False
        checks.append(("心跳套接字路径可绑定", is_sock, cfg.socket_path))
    # 锁文件路径由 socket_path 派生；目录不必已存在，看门狗启动时会 makedirs
    sock_dir = os.path.dirname(cfg.socket_path)
    probe = sock_dir
    while probe and not os.path.isdir(probe):
        parent = os.path.dirname(probe)
        if parent == probe:
            break
        probe = parent
    writable = os.path.isdir(probe) and os.access(probe, os.W_OK)
    checks.append(("锁文件/套接字目录可创建", writable, sock_dir))

    lock_path = cfg.lock_path
    if os.path.exists(lock_path):
        try:
            fd = os.open(lock_path, os.O_RDWR)
        except OSError as e:
            checks.append(("锁文件可打开", False, f"{lock_path}: {e}"))
        else:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                checks.append(("锁文件未被占用", False, lock_path))
            else:
                fcntl.flock(fd, fcntl.LOCK_UN)
                checks.append(("锁文件可用", True, lock_path))
            finally:
                os.close(fd)

    if cfg.log_file:
        log_dir = os.path.dirname(cfg.log_file)
        probe = log_dir
        while probe and not os.path.isdir(probe):
            parent = os.path.dirname(probe)
            if parent == probe:
                break
            probe = parent
        writable = os.path.isdir(probe) and os.access(probe, os.W_OK)
        checks.append(("日志目录可创建", writable, log_dir))

    for name, passed, detail in checks:
        print(f"  [{'OK' if passed else '!!'}] {name}: {detail}")
        ok = ok and passed

    if not cfg.heartbeat_enabled:
        print("\n提示: heartbeat_enabled=false，看门狗只按进程存活重启，"
              "无法发现主程序卡死。")
    else:
        print("\n提示: 心跳需要主程序本次构建包含 common/Debug/WatchdogFeed"
              "（否则首次喂狗会超时，看门狗将反复重启）。")
    print("\n结论:", "全部通过" if ok else "存在问题（见上面 [!!] 项）")
    return 0 if ok else 1


def indent(text, prefix="  "):
    return "\n".join(prefix + line for line in text.splitlines())


SELF_TEST_FEEDER = '''#!/usr/bin/env python3
# watchdog.py --self-test 使用的假“主程序”：喂狗 N 秒后停喂（模拟卡死），进程继续活着
import os, signal, socket, sys, time
signal.signal(signal.SIGINT, signal.SIG_IGN)   # 故意忽略 SIGINT，验证看门狗会升级到 SIGKILL
path = os.environ["WATCHDOG_SOCKET_PATH"]
fed = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
s.connect(path)
print("[selftest-feeder] 已连接看门狗，开始喂狗 %.0fs" % fed, flush=True)
t0, i = time.time(), 0
while True:
    if time.time() - t0 < fed:
        i += 1
        try:
            s.send(("S2C-WD/1 FEED seq=%d frames=%d\\n" % (i, i)).encode())
        except OSError:
            time.sleep(0.3)
            try:
                s.connect(path)
            except OSError:
                pass
    time.sleep(0.2)
'''


def self_test(cfg, log, args):
    """--self-test：不需要串口设备，用内置假程序验证整条链路（约 20s）"""
    fd, feeder = tempfile.mkstemp(prefix="s2c-watchdog-selftest-", suffix=".py")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(SELF_TEST_FEEDER)
    try:
        child_cmd = " ".join([shlex.quote(sys.executable), shlex.quote(feeder), "2"])
        argv = [sys.executable, os.path.abspath(__file__),
                "--config", cfg.path,
                "--command", child_cmd,
                "--heartbeat",                 # 自检必须验心跳，无视配置里的开关
                "--feed-timeout", "2",
                "--connect-timeout", "6",
                "--feed-period", "200",
                "--stall-timeout", "1000",
                "--boot-grace", "1000",
                "--max-restarts", "1",
                "--no-log",
                "--log-level", "info"]
        print("[watchdog] self-test：用假程序验证 心跳 → 卡死判定 → 杀进程组 → 重启")
        print("[watchdog] 执行: " + " ".join(shlex.quote(a) for a in argv[1:]))
        print("-" * 72)
        proc = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True)
        out = proc.stdout or ""
        print(out, end="")
        print("-" * 72)

        checks = [
            ("心跳套接字建立并收到喂狗", "[selftest-feeder] 已连接看门狗" in out),
            ("喂狗超时判定为卡死", "判定主程序卡死" in out),
            ("整组 SIGINT/SIGKILL 清理（触发 SIGKILL 升级）", "已强制终止" in out),
            ("退避后重启子进程", "第 1 次重启" in out),
            ("max_restarts 生效并干净退出", proc.returncode == 2),
            ("无残留：套接字已清理", not os.path.exists(cfg.socket_path)),
        ]
        ok = True
        for name, passed in checks:
            print(f"  [{'OK' if passed else '!!'}] {name}")
            ok = ok and passed
        print("\nself-test 结论:", "全部通过" if ok else "存在失败项")
        if not ok:
            print("提示: 若第一项就失败，检查 socket_path 所在目录权限；"
                  "若日志出现“已有看门狗实例在运行”，先停掉在跑的看门狗（或 systemd 服务）再自检。")
        return 0 if ok else 1
    finally:
        try:
            os.unlink(feeder)
        except OSError:
            pass


def build_arg_parser():
    p = argparse.ArgumentParser(
        prog="watchdog.py",
        description="串口通信服务端看门狗（配置见同目录 auto_launch.conf）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=None,
                   help=f"配置文件路径（默认 {DEFAULT_CONFIG_NAME}，与脚本同目录）")
    p.add_argument("--run", action="store_true",
                   help="不看门狗，只在项目根目录前台运行配置中的命令行")
    p.add_argument("--check", action="store_true", help="只检查配置与环境，不启动")
    p.add_argument("--self-test", action="store_true",
                   help="用内置假程序验证心跳/超时/杀进程组/重启链路（约 20s，不需要串口设备）")
    p.add_argument("--print-command", action="store_true",
                   help="打印将执行的 shell 脚本（配合 --run/--check 使用，或单独使用）")
    p.add_argument("--command", default=None, help="覆盖配置中的运行命令行（调试用）")
    p.add_argument("--no-heartbeat", action="store_true", help="关闭心跳判定，只看进程存活")
    p.add_argument("--heartbeat", action="store_true",
                   help="强制开启心跳判定（覆盖配置，调试/自检用）")
    p.add_argument("--feed-timeout", default=None, help="覆盖 feed_timeout_sec")
    p.add_argument("--connect-timeout", default=None, help="覆盖 connect_timeout_sec")
    p.add_argument("--max-restarts", default=None, help="覆盖 max_restarts（调试用）")
    p.add_argument("--socket-path", default=None,
                   help="覆盖配置中的心跳套接字路径（调试/测试用）")
    p.add_argument("--feed-period", default=None, help="覆盖 feed_period_ms")
    p.add_argument("--stall-timeout", default=None, help="覆盖 stall_timeout_ms")
    p.add_argument("--boot-grace", default=None, help="覆盖 boot_grace_ms")
    p.add_argument("--log-file", default=None, help="覆盖日志文件路径（相对项目根）")
    p.add_argument("--no-log", action="store_true", help="不写日志文件，只输出到 stdout")
    p.add_argument("--log-level", default=None, help="覆盖日志级别 debug/info/warn/error")
    return p


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    conf_path = args.config or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                           DEFAULT_CONFIG_NAME)
    cli = {
        "command": args.command,
        "no_heartbeat": args.no_heartbeat,
        "heartbeat": args.heartbeat,
        "feed_timeout": args.feed_timeout,
        "connect_timeout": args.connect_timeout,
        "max_restarts": args.max_restarts,
        "log_file": args.log_file,
        "no_log": args.no_log,
        "log_level": args.log_level,
        "socket_path": args.socket_path,
        "feed_period": args.feed_period,
        "stall_timeout": args.stall_timeout,
        "boot_grace": args.boot_grace,
    }

    try:
        cfg = Config(conf_path, cli)
    except ConfigError as e:
        sys.stderr.write(f"[watchdog] 配置错误: {e}\n")
        return 2

    # 日志级别：--check/--self-test/--print-command 时也照常输出到 stdout，但不建日志文件
    log = Logger(cfg.log_level,
                 cfg.log_file if not (args.check or args.self_test or
                                      (args.print_command and not args.run)) else None,
                 cfg.log_max_bytes)
    try:
        if args.print_command and not args.run:
            print(f"# 项目根目录: {cfg.project_root}")
            print(f"# shell: {cfg.shell}")
            print(cfg.shell_script(), end="")
            if not (args.check or args.self_test):
                return 0
        if args.check:
            return check(cfg, log, args)
        if args.self_test:
            return self_test(cfg, log, args)
        if args.run:
            return run_foreground(cfg, log, args)

        log.info("=" * 72)
        log.info("串口通信服务端看门狗启动")
        log.info("\n%s", cfg.summary())
        wd = Watchdog(cfg, log, args)
        wd.install_signal_handlers()
        try:
            return wd.run()
        except BaseException:
            # 未捕获异常：先清掉子进程组，避免看门狗退出后主程序失去管理
            wd.kill_child("未捕获异常")
            raise
        finally:
            wd.release()
    except KeyboardInterrupt:
        log.warn("收到中断信号，退出")
        return 0
    except ConfigError as e:
        log.error("%s", e)
        return 2
    except OSError as e:
        log.error("运行错误: %s", e)
        return 1
    finally:
        log.close()


if __name__ == "__main__":
    sys.exit(main())
