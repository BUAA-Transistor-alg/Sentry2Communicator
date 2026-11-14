#!/usr/bin/env python3
# test_watchdog_heartbeat.py — 验证看门狗只把合法 FEED 报文计入存活
import importlib.util
import os
import socket
import sys
import time
import types


def main():
    build_dir = sys.argv[1]
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    watchdog_path = os.path.join(root, "auto_launch", "watchdog.py")
    spec = importlib.util.spec_from_file_location("s2c_watchdog_under_test", watchdog_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    sock_path = os.path.join(build_dir, "heartbeat_test.sock")
    try:
        os.unlink(sock_path)
    except OSError:
        pass

    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(sock_path)
    server.setblocking(False)

    watchdog = module.Watchdog.__new__(module.Watchdog)
    watchdog.sock = server
    watchdog.last_feed = None
    watchdog.heartbeat_stat = ""
    watchdog.log = types.SimpleNamespace(debug=lambda *args, **kwargs: None)

    client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        client.sendto(b"arbitrary bytes, not a feed", sock_path)
        time.sleep(0.05)
        if watchdog.drain_heartbeats():
            print("FAIL: 非法报文被计入存活")
            return 1
        if watchdog.last_feed is not None:
            print("FAIL: 非法报文刷新了 last_feed")
            return 1

        client.sendto(b"S2C-WD/1 FEED seq=7 frames=70\n", sock_path)
        deadline = time.time() + 1.0
        accepted = False
        while time.time() < deadline:
            if watchdog.drain_heartbeats():
                accepted = True
                break
            time.sleep(0.01)
        if not accepted or watchdog.last_feed is None:
            print("FAIL: 合法心跳未被接受")
            return 1
        if watchdog.heartbeat_stat != "frames=70":
            print("FAIL: 心跳内容未按协议解析: %r" % watchdog.heartbeat_stat)
            return 1
    finally:
        client.close()
        server.close()
        try:
            os.unlink(sock_path)
        except OSError:
            pass

    print("watchdog heartbeat validation test passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
