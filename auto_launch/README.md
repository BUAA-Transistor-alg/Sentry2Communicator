# auto_launch — 串口通信服务端自启动与看门狗

让 `s2c_serial_comm_server`（主工程 `src/serial_comm_server.cpp`，运行 `SerialCommServer`）
在机器人上「上电即启动、崩溃后自动重启、卡死也能自动重启」。

脚本复制自 `/home/huhu233/rm2027/UnifiedAutoAimPipeline/auto_launch`，只改了配置与名称；
与那个项目同时运行时使用的共享内存、信号量、看门狗套接字、systemd 服务名都不相同。

## 文件

| 文件 | 作用 |
| --- | --- |
| `auto_launch.conf` | 唯一配置：运行命令、看门狗参数、systemd 服务参数 |
| `watchdog.py` | 看门狗：起子进程、进程级 + 心跳级监控、退避重启、整组清理 |
| `install_service.py` | 安装 / 卸载 / 状态 / 日志（systemd 系统服务，开机自启） |

心跳由主程序里的 `WatchdogFeed`（`include/common/Debug/WatchdogFeed.h`）上报；
`SerialCommServer` 的 500Hz 发送循环每轮调用一次 `tick()`，发送循环停摆即停止喂狗，
看门狗在 `feed_timeout_sec` 后判定卡死并重启整个进程组。

## 构建与自检

```bash
bash build.sh

# 不启动服务，只检查配置与环境
./auto_launch/watchdog.py --config auto_launch/auto_launch.conf --check

# 用内置假程序验证「心跳 → 卡死判定 → 杀进程组 → 重启」链路（约 10s，不需要串口设备）
./auto_launch/watchdog.py --config auto_launch/auto_launch.conf --self-test

# 只在前台运行看门狗（不装 systemd）
./auto_launch/watchdog.py --config auto_launch/auto_launch.conf
```

## 安装为开机自启服务

```bash
./auto_launch/install_service.py            # 安装并启动（需要 sudo）
./auto_launch/install_service.py status     # 状态
./auto_launch/install_service.py logs       # journalctl 最近 200 行
./auto_launch/install_service.py stop       # 停止
./auto_launch/install_service.py uninstall  # 卸载（保留项目文件与日志）
```

对应 unit：`/etc/systemd/system/s2c_serial_comm.service`，默认运行用户为 sudo 调用者，
工作目录为项目根，`ExecStart` 为 `/usr/bin/python3 <项目>/auto_launch/watchdog.py --config ...`。
改完 `auto_launch.conf` 后：

```bash
sudo systemctl reload s2c_serial_comm.service    # 重读配置并重启子进程
sudo systemctl restart s2c_serial_comm.service
```

## 与 UnifiedAutoAimPipeline 同时运行的隔离项

| 项 | 本工程 | UnifiedAutoAimPipeline |
| --- | --- | --- |
| 共享内存 | POSIX `/s2c_serial_comm` | SysV shm（config 的 `shm_key`） |
| 具名信号量 | 无（信号量内嵌在共享内存里） | `/uap_infer_req_<key>`、`/uap_infer_resp_<key>` |
| 看门狗套接字 | `/tmp/s2c_serial_comm_watchdog.sock` | `/tmp/unified_auto_aim_watchdog.sock` |
| systemd 服务 | `s2c_serial_comm.service` | `unified_auto_aim.service` |

串口是独占资源：`s2c_serial_comm_server` 打开电控与 IMU 串口后，另一个进程不能再打开同一设备；
视觉主程序通过 `SerialCommClient` 读写这份数据。

## 看门狗行为

- 进程级：子进程（整个进程组）退出后按 `restart_policy` 重启，连续快速失败指数退避；
- 心跳级：`WatchdogFeed` 每 `feed_period_ms` 上报一次；`SerialCommServer` 每轮 500Hz 循环
  `tick()`，连续 `stall_timeout_ms` 没有新帧就停止喂狗，看门狗在 `feed_timeout_sec` 后重启；
- 退出：SIGTERM/SIGINT → 对进程组 SIGINT → 宽限后 SIGKILL；SIGHUP 重读配置；
- 日志：journal + `logs/watchdog.log`（按大小轮转）。
