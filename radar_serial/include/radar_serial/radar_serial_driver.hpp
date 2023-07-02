// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RADAR_SERIAL_DRIVER__RADAR_SERIAL_DRIVER_HPP_
#define RADAR_SERIAL_DRIVER__RADAR_SERIAL_DRIVER_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace radar_serial_driver
{
  class RadarSerialDriver : public rclcpp::Node
  {
  public:
    explicit RadarSerialDriver(const rclcpp::NodeOptions &options);

    ~RadarSerialDriver() override;

  private:
    void getParams();

    void receiveData();

    void sendData();

    
    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    std::thread receive_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
  }; // namespace rm_serial_driver
}

#endif // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
