#include <termio.h>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "radar_interfaces/msg/keyboard.hpp"

namespace keyboard
{
  int scanKeyboard()
  {

    int in;

    struct termios new_settings;
    struct termios stored_settings;
    // 设置终端参数
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    in = getchar();
    tcsetattr(0, TCSANOW, &stored_settings);

    return in;
  }

  class PublisherNode : public rclcpp::Node
  {
  public:
    PublisherNode() : Node("keypub_node")
    {
      parameters_client =
          std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_parameter_server");
      parameters_client->wait_for_service();

      publisher_ = this->create_publisher<radar_interfaces::msg::Keyboard>("keyboard", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&PublisherNode::publishMessage, this));
    }
  
  private:
    void publishMessage()
    {
      auto message = radar_interfaces::msg::Keyboard();
      message.keynum = scanKeyboard();

      if (message.keynum == 106)
        parameters.push_back(rclcpp::Parameter("status", 0));
      else if (message.keynum == 107)
        parameters.push_back(rclcpp::Parameter("status", 1));

      parameters_client->set_parameters(parameters);

      publisher_->publish(message);
    }

    rclcpp::Publisher<radar_interfaces::msg::Keyboard>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 全局参数
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
    std::vector<rclcpp::Parameter> parameters;

    int keynum;
  };
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<keyboard::PublisherNode>());
    rclcpp::shutdown();
    return 0;
}