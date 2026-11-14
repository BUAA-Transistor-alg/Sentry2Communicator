# ipc_client — 本机进程间通信客户端与串口通信客户端（独立 CMake 工程）

客户端类 `IpcClient` 只提供两个方法:

- `bool get(uint32_t channel, T& value)`：读取服务端提前设定好的内容
- `bool send(uint32_t channel, const T& value)`：把内容发送给服务端，服务端在接收到时调用回调

通信基于共享内存、信号量与进程共享 robust 互斥量。

本工程自包含：全部头文件都在 `include/` 下，不从主工程或任何外部目录引用文件。
主工程反向引用这里的共享定义。

对外唯一接口是 [`include/SerialCommClient.hpp`](include/SerialCommClient.hpp)，外部工程只需包含它。

| 头文件 | 内容 | 性质 |
| --- | --- | --- |
| `include/SerialCommClient.hpp` | 串口通信客户端类（`vis_` / `nav_` 方法） | 对外接口 |
| `include/serial_comm/SerialCommProtocol.hpp` | MCU / IMU 串口协议包与裁判系统数据段定义 | 内部 |
| `include/serial_comm/SerialCommTypes.hpp` | 串口通信服务端与客户端共用的内容结构体、通道编号 | 内部 |
| `include/ipc/IpcLayout.hpp` | 共享内存布局、通道与队列结构、robust 互斥量加锁封装 | 内部 |
| `include/ipc_client/IpcClient.hpp` | 本机进程间通信客户端类 | 内部 |

## 构建

```bash
cmake -S ipc_client -B ipc_client/build
cmake --build ipc_client/build
```

产物：`ipc_client/build/libipc_client.a`、`ipc_client/build/ipc_client_example`。

## 运行示例

先构建主工程（`bash build.sh`），启动串口通信服务端：

```bash
./build/s2c_serial_comm_server
```

再开多个终端运行客户端示例（示例使用 `SerialCommClient` 读写 `/s2c_serial_comm`）：

```bash
./ipc_client/build/ipc_client_example
```

## 并发与返回时间

- 多个客户端进程可以同时使用同一个服务端；获取通道与请求队列由进程共享 robust 互斥量保护
- 客户端不等待信号量、不设置任何超时：`get` 直接读共享内存，`send` 入队后立即返回
- 服务端进程死亡时，内核把其持有的 robust 互斥量标记为持有者死亡，下一个加锁者接管；
  客户端每次调用还会检查服务端状态与进程存活，失效时直接返回 `false`；
  `get` 与 `stop()` 恰好并发、或存活检查通过后服务端立即被杀时，可能返回一次停止前的完整快照；
  下一次调用返回 `false`
- `AllInfo.send_data` 是写入请求值（`McuDataPreprocessor::processSend` 之前），
  串口实际发送值可能经过预处理器映射
- 请求队列满时覆盖最旧的一条请求，客户端不会因服务端不消费而阻塞
- `send` 是尽力入队：与服务端 `stop()` 并发时可能返回 `true` 但请求随后被丢弃

## 串口通信客户端

主工程的 `s2c::serial_comm::SerialCommServer` 维护要发给串口的数据与串口收到的最新数据，
按 300Hz 发往电控；本工程通过唯一对外接口 `SerialCommClient.hpp` 提供两部分方法:

- `bool vis_get_all_info(AllInfo& info)` / `bool nav_get_all_info(AllInfo& info)`：读取完整快照
  （`AllInfo` 含 6 个结构体：要发送的数据、最新电控数据、最新 IMU 数据、
  视觉向导航通信、导航向视觉通信、服务端信息）
- `bool vis_write(const VisWrite& write)`：写入火控、四个轴的控制部分与视觉向导航通信
- `bool nav_write(const NavWrite& write)`：写入变形与底盘位移与导航向视觉通信

内容类型与通道编号定义在本工程 `include/serial_comm/SerialCommTypes.hpp`，服务端与客户端
共用同一份声明；共享内存名字默认为 `/s2c_serial_comm`。
