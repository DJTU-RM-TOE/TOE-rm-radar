// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef __RADAR_SERIAL_CRC_HPP__
#define __RADAR_SERIAL_CRC_HPP__

#include <cstdint>

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

#endif  
