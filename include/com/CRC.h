// CRC.h — 帧尾校验函数声明
//
//   CRC8  : MCU 帧尾（查表法，初值 0xFF）
//   CRC32 : IMU 帧尾（STM32 HAL 兼容，多项式 0x04C11DB7，按 32 位字计算）
#ifndef S2C_COM_CRC_H
#define S2C_COM_CRC_H

#include <stdint.h>
#include <cstddef>

namespace s2c {
namespace com {

// CRC8 查表法表格声明
extern const uint8_t CRC8_TAB[256];

// CRC8 校验函数声明（标准化签名：const uint8_t*, size_t）
uint8_t CRC8_Check_Sum(const uint8_t *pchMessage, size_t dwLength);

// CRC32 校验函数声明（STM32 HAL 兼容）
uint32_t CRC32_Calculate(const uint8_t* data, size_t length);

}  // namespace com
}  // namespace s2c

#endif // S2C_COM_CRC_H
