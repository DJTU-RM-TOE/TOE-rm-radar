// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "radar_serial/radar_serial_driver.hpp"

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "radar_serial/crc.hpp"
#include "radar_serial/packet.hpp"

namespace radar_serial_driver
{
  RadarSerialDriver::RadarSerialDriver(const rclcpp::NodeOptions &options)
      : Node("radar_serial_driver", options),
        owned_ctx_{new IoContext(2)},
        serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
  {
    RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

    getParams();

    try
    {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open())
      {
        serial_driver_->port()->open();
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(
          get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
      throw ex;
    }

    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&RadarSerialDriver::sendData, this));
  }

  RadarSerialDriver::~RadarSerialDriver()
  {
    if (serial_driver_->port()->is_open())
    {
      serial_driver_->port()->close();
    }

    if (owned_ctx_)
    {
      owned_ctx_->waitForExit();
    }
  }

  void RadarSerialDriver::receiveData()
  {
  }

  void RadarSerialDriver::sendData()
  {
    SendPacket packet;

    packet.CRC8 = Get_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&packet), 4, 0xff);
    packet.CRC16 = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - 2, 0xffff);

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);
  }

  void RadarSerialDriver::getParams()
  {
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate{};

    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    baud_rate = 115200;

    device_config_ =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

    device_name_ = "/dev/ttyUSB0";
  }

} // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(radar_serial_driver::RadarSerialDriver);