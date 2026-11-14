#!/usr/bin/env python3
# test_watchdog_reload.py — 验证 SIGHUP 重载 socket_path 时套接字与锁文件一起迁移
import fcntl
import importlib.util
import os
import shutil
import sys
import tempfile
import types


def write_config(path, project_root, socket_path):
    with open(path, "w", encoding="utf-8") as f:
        f.write("""[launch]
command = /bin/true
preamble =
shell = /bin/bash
project_root = {root}
restart_policy = always

[watchdog]
heartbeat_enabled = true
socket_path = {sock}
feed_timeout_sec = 5
connect_timeout_sec = 20
shutdown_grace_sec = 1
restart_delay_sec = 1
restart_backoff_max_sec = 5
healthy_run_sec = 10
max_restarts = 0
feed_period_ms = 200
stall_timeout_ms = 2000
boot_grace_ms = 1000
log_file =
log_max_bytes = 1024
log_level = info
status_log_sec = 10

[service]
name = s2c_reload_test.service
description = reload test
after =
wants =
environment =
restart_sec = 1
timeout_stop_sec = 5
""".format(root=project_root, sock=socket_path))


def lock_is_held(path):
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return False
    except OSError:
        return True
    finally:
        os.close(fd)


def main():
    build_dir = sys.argv[1]
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    module_path = os.path.join(root, "auto_launch", "watchdog.py")
    spec = importlib.util.spec_from_file_location("s2c_watchdog_reload_test", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    work = tempfile.mkdtemp(prefix="s2c-reload-", dir=build_dir)
    config_path = os.path.join(work, "test.conf")
    socket_a = os.path.join(work, "a.sock")
    socket_b = os.path.join(work, "b.sock")
    write_config(config_path, root, socket_a)

    log = types.SimpleNamespace(
        debug=lambda *args, **kwargs: None,
        info=lambda *args, **kwargs: None,
        warn=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
        reconfigure=lambda *args, **kwargs: None,
    )
    args = types.SimpleNamespace(
        command=None, no_heartbeat=False, heartbeat=False,
        feed_timeout=None, connect_timeout=None, max_restarts=None,
        log_file=None, no_log=True, log_level=None, socket_path=None,
        feed_period=None, stall_timeout=None, boot_grace=None,
    )
    cfg = module.Config(config_path, None)

    watchdog = module.Watchdog.__new__(module.Watchdog)
    watchdog.cfg = cfg
    watchdog.log = log
    watchdog.args = args
    watchdog.child = None
    watchdog.sock = None
    watchdog.socket_created = False
    watchdog.lock_fd = None
    watchdog.stop_requested = False
    watchdog.reload_requested = False
    watchdog.started_at = 0.0
    watchdog.spawn_time = 0.0
    watchdog.last_feed = None
    watchdog.heartbeat_stat = ""
    watchdog.restarts = 0
    watchdog.failures = 0

    watchdog.acquire_lock()
    watchdog.open_socket()
    if not os.path.exists(socket_a):
        print("FAIL: 初始套接字未建立")
        return 1

    write_config(config_path, root, socket_b)
    watchdog.reload_config()

    failures = []
    if os.path.exists(socket_a):
        failures.append("旧套接字未删除")
    if not os.path.exists(socket_b):
        failures.append("新套接字未建立")
    if not lock_is_held(socket_b + ".lock"):
        failures.append("新锁文件未被持有")
    if lock_is_held(socket_a + ".lock"):
        failures.append("旧锁文件未释放")
    if not watchdog.lock_fd:
        failures.append("锁文件描述符未更新")

    watchdog._close_socket()
    watchdog._release_lock_fd(watchdog.lock_fd)
    watchdog.lock_fd = None
    shutil.rmtree(work, ignore_errors=True)

    if failures:
        print("FAIL: " + "; ".join(failures))
        return 1
    print("watchdog reload lock migration test passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
