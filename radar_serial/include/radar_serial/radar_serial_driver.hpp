// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RADAR_SERIAL_DRIVER__RADAR_SERIAL_DRIVER_HPP_
#define RADAR_SERIAL_DRIVER__RADAR_SERIAL_DRIVER_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

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

    void listenTf();

    void callbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future);

    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::thread receive_thread_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped transformStamped_num[2][6];
    
    int sequence_flag = 0;
    
    //全局参数
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
    std::vector<rclcpp::Parameter> parameters;

  }; // namespace rm_serial_driver
}

#endif // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
