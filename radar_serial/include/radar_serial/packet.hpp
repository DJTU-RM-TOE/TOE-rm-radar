// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RADAR_SERIAL_DRIVER__PACKET_HPP_
#define RADAR_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace radar_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t robot_color : 1;
  uint8_t task_mode : 2;
  uint8_t reserved : 5;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
    uint8_t SOF = (uint8_t)0xA5;
    uint16_t data_length = 0x000A;
    uint8_t seq;
    uint8_t CRC8;

    uint16_t cmd_id = 0x0305U;

    uint16_t target_robot_ID = 0x0001U;
    float target_position_x = 1.0;
    float target_position_y = 1.0;

    uint16_t CRC16;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
