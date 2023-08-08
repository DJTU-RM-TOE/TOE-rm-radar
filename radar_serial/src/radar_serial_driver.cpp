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

#define COLOR_B 0
#define COLOR_R 1

namespace radar_serial_driver
{
  RadarSerialDriver::RadarSerialDriver(const rclcpp::NodeOptions &options)
      : Node("radar_serial_driver", options),
        owned_ctx_{new IoContext(2)},
        serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
  {
    RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

    // get param
    parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_parameter_server");
    parameters_client->wait_for_service();
    auto parameters_future = parameters_client->get_parameters({"color"},
                                                               std::bind(&RadarSerialDriver::callbackGlobalParam, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    getParams();
    RCLCPP_INFO(get_logger(), "Finish getParams");
    try
    {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open())
      {
        serial_driver_->port()->open();
        receive_thread_ = std::thread(&RadarSerialDriver::receiveData, this);
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(
          get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
      throw ex;
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&RadarSerialDriver::sendData, this));
  }

  void RadarSerialDriver::callbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future)
  {
    std::vector<rclcpp::Parameter> result = future.get();
    rclcpp::Parameter param = result.at(0);
    RCLCPP_INFO(this->get_logger(), "Got color param: %ld", param.as_int());
  }

  RadarSerialDriver::~RadarSerialDriver()
  {
    if (receive_thread_.joinable())
    {
      receive_thread_.join();
    }

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
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;
    data.reserve(sizeof(ReceivePacket));

    ReceivePacket packet;
    while (rclcpp::ok())
    {
      serial_driver_->port()->receive(header);
      if (header[0] == 0xA5)
      {
        data.resize(sizeof(ReceivePacket));
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        packet = fromVector(data);

        if (packet.cmd_id == 0x201)
        {
          uint8_t CRC8 = Get_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&packet), 4, 0xff);
          uint16_t CRC16 = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - 2, 0xffff);
          long int id = packet.message.robot_id;
          // RCLCPP_INFO(get_logger(), "id: %ld", id);
          if (id == 109)
          {
            parameters.push_back(rclcpp::Parameter("color", 1));
          }
          else if (id == 9)
          {
            parameters.push_back(rclcpp::Parameter("color", 0));
          }
          else
          {
            RCLCPP_INFO(get_logger(), "id data error");
          }
          parameters_client->set_parameters(parameters);

          if (CRC8 == packet.CRC8 && CRC16 == packet.CRC16)
          {
            // RCLCPP_INFO(get_logger(), "CRC yes!");
            //   RCLCPP_INFO(get_logger(), "%x %x %x %x",packet.CRC8,CRC8,packet.CRC16,CRC16);
            //   RCLCPP_INFO(get_logger(), "id : %x",packet.cmd_id);
          }
        }
      }
    }
  }

  void RadarSerialDriver::sendData()
  {
    SendPacket packet;

    if (sequence_flag >= 6)
    {
      sequence_flag = 0;
    }
    else
      sequence_flag++;

    RadarSerialDriver::listenTf();

    // send Blue
    float y = -(float)transformStamped_num[COLOR_B][sequence_flag].transform.translation.y + 7.5;
    float x = -(float)transformStamped_num[COLOR_B][sequence_flag].transform.translation.x + 14;

    packet.target_position_y = y;
    packet.target_position_x = x;

    packet.target_robot_ID = 101 + sequence_flag;

    //RCLCPP_INFO(get_logger(), "%f %f", transformStamped_num[COLOR_B][sequence_flag].transform.translation.x + 14, (float)transformStamped_num[COLOR_B][sequence_flag].transform.translation.y + 7.5);

    if (y > 15 || x > 28)
    {
      return;
    }

    if (packet.seq >= 255)
      packet.seq = 0;
    else
      packet.seq++;

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

  void RadarSerialDriver::listenTf()
  {
    try
    {
      transformStamped_num[COLOR_B][0] = tf_buffer_->lookupTransform("map", "RobotB1", tf2::TimePointZero);
      transformStamped_num[COLOR_B][1] = tf_buffer_->lookupTransform("map", "RobotB2", tf2::TimePointZero);
      transformStamped_num[COLOR_B][2] = tf_buffer_->lookupTransform("map", "RobotB3", tf2::TimePointZero);
      transformStamped_num[COLOR_B][3] = tf_buffer_->lookupTransform("map", "RobotB4", tf2::TimePointZero);
      transformStamped_num[COLOR_B][4] = tf_buffer_->lookupTransform("map", "RobotB5", tf2::TimePointZero);
      transformStamped_num[COLOR_B][5] = tf_buffer_->lookupTransform("map", "RobotB6", tf2::TimePointZero);
      transformStamped_num[COLOR_R][0] = tf_buffer_->lookupTransform("map", "RobotR1", tf2::TimePointZero);
      transformStamped_num[COLOR_R][1] = tf_buffer_->lookupTransform("map", "RobotR2", tf2::TimePointZero);
      transformStamped_num[COLOR_R][2] = tf_buffer_->lookupTransform("map", "RobotR3", tf2::TimePointZero);
      transformStamped_num[COLOR_R][3] = tf_buffer_->lookupTransform("map", "RobotR4", tf2::TimePointZero);
      transformStamped_num[COLOR_R][4] = tf_buffer_->lookupTransform("map", "RobotR5", tf2::TimePointZero);
      transformStamped_num[COLOR_R][5] = tf_buffer_->lookupTransform("map", "RobotR6", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      // RCLCPP_WARN(this->get_logger(), "Failed to receive transform: %s", ex.what());
    }
  }

} // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(radar_serial_driver::RadarSerialDriver);
