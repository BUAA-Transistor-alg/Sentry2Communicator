#include "SerialCommClient.hpp"

namespace s2c {
namespace serial_comm {

SerialCommClient::SerialCommClient(const std::string& name) : client_(name) {}

SerialCommClient::~SerialCommClient() = default;

bool SerialCommClient::vis_get_all_info(AllInfo& info) {
    return client_.get(kAllInfoChannel, info);
}

bool SerialCommClient::vis_write(const VisWrite& write) {
    return client_.send(kVisWriteChannel, write);
}

bool SerialCommClient::nav_get_all_info(AllInfo& info) {
    return client_.get(kAllInfoChannel, info);
}

bool SerialCommClient::nav_write(const NavWrite& write) {
    return client_.send(kNavWriteChannel, write);
}

}  // namespace serial_comm
}  // namespace s2c
