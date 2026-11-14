#include "McuDataPreprocessor.h"

namespace s2c {
namespace com {

mcu::SendPacket McuDataPreprocessor::processSend(const mcu::SendPacket& packet) {
    return packet;
}

mcu::ReceivePacket McuDataPreprocessor::processReceive(const mcu::ReceivePacket& packet) {
    return packet;
}

}  // namespace com
}  // namespace s2c
