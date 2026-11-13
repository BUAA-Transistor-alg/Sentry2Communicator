cmake_minimum_required(VERSION 3.16)

project(IpcClient
    VERSION 1.0.0
    DESCRIPTION "Sentry2Communicator 本机进程间通信客户端与串口通信客户端（共享内存 + 信号量）"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)

# 本工程自包含: 布局、协议包、串口通信内容与客户端头文件都在 include/ 下，
# 不从主工程或任何外部目录引用文件。主工程反向引用这里的共享定义。
add_library(ipc_client STATIC
    src/IpcClient.cpp
    src/SerialCommClient.cpp)

target_include_directories(ipc_client PUBLIC include)
target_link_libraries(ipc_client PUBLIC Threads::Threads)
target_compile_options(ipc_client PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ipc_client_example examples/example.cpp)
target_link_libraries(ipc_client_example PRIVATE ipc_client)
target_compile_options(ipc_client_example PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(ipc_client ipc_client_example PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
