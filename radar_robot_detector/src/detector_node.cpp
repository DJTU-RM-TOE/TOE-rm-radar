#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace radar_detector
{
  class detector_node : public rclcpp::Node
  {
  public:
    explicit detector_node(const rclcpp::NodeOptions &options) : Node("detector_node")
    {
      broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

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

      timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&detector_node::run, this));
    }

  private:
    void run()
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

/*
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<radar_detector::detector_node>());
  rclcpp::shutdown();
  return 0;
}
*/