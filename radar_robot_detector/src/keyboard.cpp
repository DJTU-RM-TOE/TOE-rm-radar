#include <termio.h>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "radar_robot_detector/keyboard.h"
#include "radar_interfaces/msg/keyboard.hpp"

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("keypub_node")
  {
    publisher_ = this->create_publisher<radar_interfaces::msg::Keyboard>("keyboard", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PublisherNode::publishMessage, this));
  }

private:
  void publishMessage()
  {
    auto message = radar_interfaces::msg::Keyboard();
    message.keynum = scanKeyboard();
    publisher_->publish(message);
  }

  rclcpp::Publisher<radar_interfaces::msg::Keyboard>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int keynum;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}