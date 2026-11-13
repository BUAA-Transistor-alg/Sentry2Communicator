cmake_minimum_required(VERSION 3.16)

project(Sentry2Communicator
    VERSION 1.0.0
    DESCRIPTION "Sentry2 上位机通信库：MCU/IMU 串口协议、数据包收发与线性映射、本机进程间通信服务端"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# 目录约定
#   include/com + src/com : 通信核心（模板化串口协议 + 数据包定义 + CRC + 线性映射），
#                           命名空间 s2c::com
#   include/ipc + src/ipc : 本机进程间通信服务端（共享内存 + 信号量 + robust 互斥量），
#                           命名空间 s2c::ipc
#   ipc_client/           : 自包含的独立 CMake 工程，本机进程间通信客户端、串口通信客户端
#                           及其使用示例；共享定义（serial_comm/SerialProtocol.hpp、
#                           serial_comm/SerialCommTypes.hpp、ipc/IpcLayout.hpp）也放在
#                           ipc_client/include，本工程反向引用这些文件
#   src/serial_comm_server.cpp : 串口通信服务端可执行程序入口
#   tools/                : 串口链路自检程序、IPC 服务端示例
#   test/                 : 测试程序（pty 对建立真实串口链路、IPC 端到端测试）
#
# 依赖: SerialProtocol 用 libudev 读 USB iProduct 以筛选串口（需 libudev），
#       并以 std::thread 做收发/重连线程（需 Threads）。
# ---------------------------------------------------------------------------
find_package(Threads REQUIRED)
find_library(S2C_UDEV_LIBRARY udev)
if(NOT S2C_UDEV_LIBRARY)
    message(FATAL_ERROR "libudev not found（include/com/SerialProtocol.hpp 依赖它筛选串口）")
endif()

add_library(s2c SHARED
    src/com/CRC.cpp
    src/com/McuDataPreprocessor.cpp
    src/ipc/IpcServer.cpp
    src/serial_comm/SerialCommServer.cpp
    src/common/Debug/WatchdogFeed.cpp
    src/common/FrameRateCounter.cpp
)

target_include_directories(s2c PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/com
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    # 共享定义（serial_comm/SerialProtocol.hpp、serial_comm/SerialCommTypes.hpp、
    # ipc/IpcLayout.hpp）
    ${CMAKE_CURRENT_SOURCE_DIR}/ipc_client/include
)

target_link_libraries(s2c PUBLIC ${S2C_UDEV_LIBRARY} Threads::Threads)

target_compile_options(s2c PRIVATE -Wall -Wextra -Wpedantic)

# 产物统一放到 build/ 根目录，便于 ./build/s2c_xxx 直接运行
set_target_properties(s2c PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

message(STATUS "s2c 动态库输出目录: ${CMAKE_BINARY_DIR}")

# ---------------------------------------------------------------------------
# 串口链路自检（打印 MCU 反馈包全部字段与实际选中端口）
# ---------------------------------------------------------------------------
add_executable(s2c_test_serial tools/test_serial.cpp)
target_link_libraries(s2c_test_serial PRIVATE s2c)
target_compile_options(s2c_test_serial PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# IPC 服务端示例（发布一条获取通道内容，收到客户端命令时打印）
# ---------------------------------------------------------------------------
add_executable(s2c_ipc_server_demo tools/ipc_server_demo.cpp)
target_link_libraries(s2c_ipc_server_demo PRIVATE s2c)
target_compile_options(s2c_ipc_server_demo PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# 串口通信服务端（打开电控与 IMU 串口，500Hz 发送并发布完整快照）
# ---------------------------------------------------------------------------
add_executable(s2c_serial_comm_server src/serial_comm_server.cpp)
target_link_libraries(s2c_serial_comm_server PRIVATE s2c)
target_compile_options(s2c_serial_comm_server PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# 测试
#   s2c_test_link                 : 串口协议层（pty 对 + 真实帧收发）
#   s2c_test_preprocessor         : 收发线性映射
#   s2c_test_robot_communication  : MCU/IMU 两路串口组合类端到端
#   s2c_test_ipc                  : IPC 端到端（服务端在本工程，客户端源码来自 ipc_client/）
#   s2c_test_serial_comm          : 串口通信服务端/客户端端到端（pty 对模拟串口）
# ---------------------------------------------------------------------------
enable_testing()

add_executable(s2c_test_link test/test_link.cpp)
target_link_libraries(s2c_test_link PRIVATE s2c)
target_compile_options(s2c_test_link PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME s2c_test_link COMMAND s2c_test_link)

add_executable(s2c_test_preprocessor test/test_preprocessor.cpp)
target_link_libraries(s2c_test_preprocessor PRIVATE s2c)
target_compile_options(s2c_test_preprocessor PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME s2c_test_preprocessor COMMAND s2c_test_preprocessor)

add_executable(s2c_test_robot_communication test/test_robot_communication.cpp)
target_link_libraries(s2c_test_robot_communication PRIVATE s2c)
target_compile_options(s2c_test_robot_communication PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME s2c_test_robot_communication COMMAND s2c_test_robot_communication)

# IPC 端到端测试: 服务端来自本工程，客户端源码来自独立工程 ipc_client/
set(IPC_CLIENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ipc_client)

add_executable(s2c_test_ipc_helper test/ipc_test_helper.cpp ${IPC_CLIENT_DIR}/src/IpcClient.cpp)
target_link_libraries(s2c_test_ipc_helper PRIVATE s2c)
target_compile_options(s2c_test_ipc_helper PRIVATE -Wall -Wextra -Wpedantic)

add_executable(s2c_test_ipc test/test_ipc.cpp ${IPC_CLIENT_DIR}/src/IpcClient.cpp)
target_link_libraries(s2c_test_ipc PRIVATE s2c)
target_compile_options(s2c_test_ipc PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(s2c_test_ipc PRIVATE
    S2C_IPC_HELPER_BIN="$<TARGET_FILE:s2c_test_ipc_helper>")
add_dependencies(s2c_test_ipc s2c_test_ipc_helper)
add_test(NAME s2c_test_ipc COMMAND s2c_test_ipc)

# 串口通信服务端/客户端端到端测试: 用 pty 对模拟电控与 IMU 串口
add_executable(s2c_test_serial_comm test/test_serial_comm.cpp
    ${IPC_CLIENT_DIR}/src/IpcClient.cpp
    ${IPC_CLIENT_DIR}/src/SerialCommClient.cpp)
target_link_libraries(s2c_test_serial_comm PRIVATE s2c)
target_compile_options(s2c_test_serial_comm PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME s2c_test_serial_comm COMMAND s2c_test_serial_comm)
