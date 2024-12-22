#include "motor_msg.h"
#include "uart.hpp"
#include <cstdint>
#include <cstring>
#include <iostream>
uint32_t data[] = {0xFEEE0001, 0x05002600, 0x70AD4241, 0xFEFFC601, 0x314C8B40,
                   0x00000000, 0x0000B0FE, 0x00007DFE, 0x14000000, 0x0000FFFF,
                   0xFFFFFFFF, 0xFFFFFFFF, 0xFFFF0000, 0x00000000, 0x00000000,
                   0x00000000, 0x00000000, 0x62F60600, 0x0000};
uint32_t crc = 0x5BFF91AC;

int main() { printf("%x", Uart::crc32_core(data, 18)); }