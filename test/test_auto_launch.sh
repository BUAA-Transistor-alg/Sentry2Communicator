#!/bin/bash
# test/test_auto_launch.sh — 看门狗 + serial_comm_server 集成测试（CTest 调用）
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [ "$#" -ge 1 ]; then
    BUILD_DIR="$(cd "$1" && pwd)"
else
    BUILD_DIR="$ROOT/build"
fi
CONFIG="$ROOT/auto_launch/auto_launch.conf"
BIN="$BUILD_DIR/s2c_serial_comm_server"
SOCK="$BUILD_DIR/auto_launch_test.sock"
LOG="$BUILD_DIR/auto_launch_test.log"
WD=0

fail() {
    echo "FAIL: $*"
    [ -f "$LOG" ] && tail -40 "$LOG"
    exit 1
}

cleanup() {
    if [ "$WD" -gt 0 ]; then
        kill -TERM "$WD" 2>/dev/null
        wait "$WD" 2>/dev/null
    fi
    rm -f "$SOCK" "$SOCK.lock"
}
trap cleanup EXIT

[ -x "$BIN" ] || fail "找不到 $BIN，请先构建"

# 共享内存名字固定：已有服务端进程时跳过，避免干扰正在运行的服务
server_running() {
    local p exe
    for p in /proc/[0-9]*; do
        exe="$(readlink "$p/exe" 2>/dev/null)" || continue
        case "$exe" in
            */s2c_serial_comm_server) return 0 ;;
        esac
    done
    return 1
}

if server_running; then
    echo "SKIP: 已有 s2c_serial_comm_server 进程在运行，跳过集成测试"
    exit 77
fi

rm -f "$SOCK" "$SOCK.lock" "$LOG"

python3 "$ROOT/auto_launch/watchdog.py" --config "$CONFIG" \
    --socket-path "$SOCK" --no-log \
    --feed-timeout 3 --connect-timeout 5 \
    --feed-period 200 --stall-timeout 1000 --boot-grace 1000 > "$LOG" 2>&1 &
WD=$!

server_pids() {
    grep -o '子进程已启动: pid=[0-9]*' "$LOG" 2>/dev/null | grep -o '[0-9]*'
}

wait_for_pid() {
    local deadline=$((SECONDS + 8))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if [ -n "$(server_pids | tail -1)" ]; then
            return 0
        fi
        sleep 0.2
    done
    return 1
}

wait_for_pid || fail "看门狗未启动服务端"
PID1="$(server_pids | tail -1)"
kill -0 "$PID1" 2>/dev/null || fail "服务端进程不存在"

# 心跳正常：6s 内不应出现卡死判定
sleep 6
grep -q '判定主程序卡死' "$LOG" && fail "心跳被误判为卡死"

# 杀掉服务端，看门狗应自动重启
kill -9 "$PID1" 2>/dev/null
deadline=$((SECONDS + 10))
PID2=""
while [ "$SECONDS" -lt "$deadline" ]; do
    PID2="$(server_pids | tail -1)"
    if [ -n "$PID2" ] && [ "$PID2" != "$PID1" ] && kill -0 "$PID2" 2>/dev/null; then
        break
    fi
    sleep 0.2
done
[ -n "$PID2" ] && [ "$PID2" != "$PID1" ] && kill -0 "$PID2" 2>/dev/null || fail "服务端未被自动重启"
grep -q '第 1 次重启' "$LOG" || fail "未观察到重启日志"

# 停止看门狗，整组清理
kill -TERM "$WD"
wait "$WD"
WD=0
trap - EXIT
kill -0 "$PID2" 2>/dev/null && fail "看门狗退出后服务端仍在运行"
[ -e "$SOCK" ] && fail "看门狗退出后套接字未清理"
rm -f "$SOCK" "$SOCK.lock"
echo "auto_launch 集成测试通过"
