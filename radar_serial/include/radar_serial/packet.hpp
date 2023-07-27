// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RADAR_SERIAL_DRIVER__PACKET_HPP_
#define RADAR_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace radar_serial_driver
{
  typedef struct
  {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_id1_17mm_barrel_cooling_value;
    uint16_t shooter_id1_17mm_barrel_heat_limit;
    uint16_t shooter_id1_17mm_initial_launching_speed_limit;
    uint16_t shooter_id2_17mm_barrel_cooling_valuecooling_rate;
    uint16_t shooter_id2_17mm_barrel_heatcooling_limit;
    uint16_t shooter_id2_17mm_initial_launching_speed_limit;
    uint16_t shooter_id1_42mm__barrel_cooling__value;
    uint16_t shooter_id1_42mm_barrel_heat_cooling_limit;
    uint16_t shooter_id1_42mm_initial_launching_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
  } __attribute__((packed)) robot_status_t;

  typedef struct
  {
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
  } __attribute__((packed)) power_heat_data_t;

  struct ReceivePacket
  {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;

    uint16_t cmd_id;

    robot_status_t message;

    uint16_t CRC16;
  } __attribute__((packed));

  struct SendPacket
  {
    uint8_t SOF = (uint8_t)0xA5;
    uint16_t data_length = 10;
    uint8_t seq;
    uint8_t CRC8;

    uint16_t cmd_id = 0x0305U;

    uint16_t target_robot_ID = 0x0001U;
    float target_position_x = 0.0;
    float target_position_y = 0.0;

    uint16_t CRC16;
  } __attribute__((packed));

  inline ReceivePacket fromVector(const std::vector<uint8_t> &data)
  {
    ReceivePacket packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
    return packet;
  }

  inline std::vector<uint8_t> toVector(const SendPacket &data)
  {
    std::vector<uint8_t> packet(sizeof(SendPacket));
    std::copy(
        reinterpret_cast<const uint8_t *>(&data),
        reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
    return packet;
  }

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__PACKET_HPP_
