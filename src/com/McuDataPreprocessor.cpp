#include "McuDataPreprocessor.h"

namespace s2c {
namespace com {

mcu::SendPacket McuDataPreprocessor::processSend(const mcu::SendPacket& packet) const {
    return packet;
}

mcu::ReceivePacket McuDataPreprocessor::processReceive(const mcu::ReceivePacket& packet) const {
    return packet;
}

}  // namespace com
}  // namespace s2c
