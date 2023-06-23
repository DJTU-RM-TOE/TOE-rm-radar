#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "radar_interfaces/msg/status.hpp"
#include "radar_interfaces/msg/keyboard.hpp"

namespace radar_detector
{
  class detector_node : public rclcpp::Node
  {
  public:
    explicit detector_node(const rclcpp::NodeOptions &options) : Node("detector_node", options)
    {
      // 标定部分
      RCLCPP_INFO(this->get_logger(), "进入标定状态");
      subscription_keyboard_ = create_subscription<radar_interfaces::msg::Keyboard>(
          "keyboard", 10, std::bind(&detector_node::keyboardCallback, this, std::placeholders::_1));

      // 状态机发布
      RCLCPP_INFO(this->get_logger(), "开始正式运行");
      // tf发布
      broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      detector_data();
      timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&detector_node::run, this));
    }

  private:
    void keyboardCallback(const radar_interfaces::msg::Keyboard::SharedPtr msg)
    {
      if (status_flag != 0)
        return;
      if (msg->keynum == 113 && point_select < 3)
        point_select++;
      if (msg->keynum == 101 && point_select > 0)
        point_select--;
      if (msg->keynum == 119 && point_x[point_select] < 1280)
        point_x[point_select]++;
      if (msg->keynum == 115 && point_x[point_select] > 1280)
        point_x[point_select]--;
      if (msg->keynum == 97 && point_y[point_select] < 1024)
        point_y[point_select]++;
      if (msg->keynum == 100 && point_y[point_select] > 1024)
        point_y[point_select]--;
      if (msg->keynum == 10)
      {
        status_flag = 1;
        publisher_ = this->create_publisher<radar_interfaces::msg::Status>("radar_status", 10);
        message_.status = status_flag;
        publisher_->publish(message_);
      }
    }

    void detector_data()
    {
      transformStamped_b1.header.frame_id = "map";
      transformStamped_b1.child_frame_id = "RobotB1";
      transformStamped_b1.transform.translation.x = -6.0;
      transformStamped_b1.transform.translation.y = 0.0;
      transformStamped_b1.transform.translation.z = 0.0;

      transformStamped_b2.header.frame_id = "map";
      transformStamped_b2.child_frame_id = "RobotB2";
      transformStamped_b2.transform.translation.x = -5.0;
      transformStamped_b2.transform.translation.y = 0.0;
      transformStamped_b2.transform.translation.z = 0.0;

      transformStamped_b3.header.frame_id = "map";
      transformStamped_b3.child_frame_id = "RobotB3";
      transformStamped_b3.transform.translation.x = -4.0;
      transformStamped_b3.transform.translation.y = 0.0;
      transformStamped_b3.transform.translation.z = 0.0;

      transformStamped_b4.header.frame_id = "map";
      transformStamped_b4.child_frame_id = "RobotB4";
      transformStamped_b4.transform.translation.x = -3.0;
      transformStamped_b4.transform.translation.y = 0.0;
      transformStamped_b4.transform.translation.z = 0.0;

      transformStamped_b5.header.frame_id = "map";
      transformStamped_b5.child_frame_id = "RobotB5";
      transformStamped_b5.transform.translation.x = -2.0;
      transformStamped_b5.transform.translation.y = 0.0;
      transformStamped_b5.transform.translation.z = 0.0;

      transformStamped_b6.header.frame_id = "map";
      transformStamped_b6.child_frame_id = "RobotB6";
      transformStamped_b6.transform.translation.x = -1.0;
      transformStamped_b6.transform.translation.y = 0.0;
      transformStamped_b6.transform.translation.z = 0.0;

      transformStamped_r1.header.frame_id = "map";
      transformStamped_r1.child_frame_id = "RobotR1";
      transformStamped_r1.transform.translation.x = 1.0;
      transformStamped_r1.transform.translation.y = 0.0;
      transformStamped_r1.transform.translation.z = 0.0;

      transformStamped_r2.header.frame_id = "map";
      transformStamped_r2.child_frame_id = "RobotR2";
      transformStamped_r2.transform.translation.x = 2.0;
      transformStamped_r2.transform.translation.y = 0.0;
      transformStamped_r2.transform.translation.z = 0.0;

      transformStamped_r3.header.frame_id = "map";
      transformStamped_r3.child_frame_id = "RobotR3";
      transformStamped_r3.transform.translation.x = 3.0;
      transformStamped_r3.transform.translation.y = 0.0;
      transformStamped_r3.transform.translation.z = 0.0;

      transformStamped_r4.header.frame_id = "map";
      transformStamped_r4.child_frame_id = "RobotR4";
      transformStamped_r4.transform.translation.x = 4.0;
      transformStamped_r4.transform.translation.y = 0.0;
      transformStamped_r4.transform.translation.z = 0.0;

      transformStamped_r5.header.frame_id = "map";
      transformStamped_r5.child_frame_id = "RobotR5";
      transformStamped_r5.transform.translation.x = 5.0;
      transformStamped_r5.transform.translation.y = 0.0;
      transformStamped_r5.transform.translation.z = 0.0;

      transformStamped_r6.header.frame_id = "map";
      transformStamped_r6.child_frame_id = "RobotR6";
      transformStamped_r6.transform.translation.x = 6.0;
      transformStamped_r6.transform.translation.y = 0.0;
      transformStamped_r6.transform.translation.z = 0.0;
    }

    void run()
    {
      if (status_flag == 1)
      {
        transformStamped_b1.header.stamp = now();
        transformStamped_b2.header.stamp = now();
        transformStamped_b3.header.stamp = now();
        transformStamped_b4.header.stamp = now();
        transformStamped_b5.header.stamp = now();
        transformStamped_b6.header.stamp = now();

        transformStamped_r1.header.stamp = now();
        transformStamped_r2.header.stamp = now();
        transformStamped_r3.header.stamp = now();
        transformStamped_r4.header.stamp = now();
        transformStamped_r5.header.stamp = now();
        transformStamped_r6.header.stamp = now();

        broadcaster_->sendTransform(transformStamped_b1);
        broadcaster_->sendTransform(transformStamped_b2);
        broadcaster_->sendTransform(transformStamped_b3);
        broadcaster_->sendTransform(transformStamped_b4);
        broadcaster_->sendTransform(transformStamped_b5);
        broadcaster_->sendTransform(transformStamped_b6);

        broadcaster_->sendTransform(transformStamped_r1);
        broadcaster_->sendTransform(transformStamped_r2);
        broadcaster_->sendTransform(transformStamped_r3);
        broadcaster_->sendTransform(transformStamped_r4);
        broadcaster_->sendTransform(transformStamped_r5);
        broadcaster_->sendTransform(transformStamped_r6);
      }
    }
    rclcpp::Subscription<radar_interfaces::msg::Keyboard>::SharedPtr subscription_keyboard_;

    int point_select = 0;

    int point_x[4] = {100, 200, 200, 100};
    int point_y[4] = {100, 100, 200, 200};

    int status_flag = 0;

    // 状态机发布
    rclcpp::Publisher<radar_interfaces::msg::Status>::SharedPtr publisher_;
    radar_interfaces::msg::Status message_;
    //

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    geometry_msgs::msg::TransformStamped transformStamped_b1;
    geometry_msgs::msg::TransformStamped transformStamped_b2;
    geometry_msgs::msg::TransformStamped transformStamped_b3;
    geometry_msgs::msg::TransformStamped transformStamped_b4;
    geometry_msgs::msg::TransformStamped transformStamped_b5;
    geometry_msgs::msg::TransformStamped transformStamped_b6;
    geometry_msgs::msg::TransformStamped transformStamped_r1;
    geometry_msgs::msg::TransformStamped transformStamped_r2;
    geometry_msgs::msg::TransformStamped transformStamped_r3;
    geometry_msgs::msg::TransformStamped transformStamped_r4;
    geometry_msgs::msg::TransformStamped transformStamped_r5;
    geometry_msgs::msg::TransformStamped transformStamped_r6;

    rclcpp::TimerBase::SharedPtr timer_;
  };
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(radar_detector::detector_node);
