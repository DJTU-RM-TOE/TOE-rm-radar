#ifndef __DETECTOR_NODE__
#define __DETECTOR_NODE__

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "radar_orientation/calibration.hpp"
#include "radar_orientation/pnp_solver.hpp"

namespace radar_orientation
{
  // 全局状态标志
  int status_flag = 0;
  int color_flag = 0;
  int point_select = 0;
  // 主节点类
  class OrientationNode : public rclcpp::Node
  {
  public:
    OrientationNode(const rclcpp::NodeOptions &options);

  private:
    // 标定类
    calibration calibration_module[2];
    // pnp
    pnp_solver pnp_solver_module1;
    pnp_solver pnp_solver_module2;

    // 全局参数读取
    void paramCallback(const radar_interfaces::msg::GlobalParam::SharedPtr msg);

    //键盘接收回调
    rclcpp::Subscription<radar_interfaces::msg::Keyboard>::SharedPtr subscription_keyboard_;
    void keyboardCallback(const radar_interfaces::msg::Keyboard::SharedPtr msg);
    int speed = 1;

    //
    void detector1(const radar_interfaces::msg::RobotFlag::SharedPtr msg);
    void detector2(const radar_interfaces::msg::RobotFlag::SharedPtr msg);
    
    double rvecArray[3];
    double tvecArray[3];

    void detector_data();
    void send_tf();
    bool pointInPolygon(cv::Point2f point, const std::vector<cv::Point2f> &polygon);

    rclcpp::Subscription<radar_interfaces::msg::RobotFlag>::SharedPtr subscription_robotflag_1_;
    rclcpp::Subscription<radar_interfaces::msg::RobotFlag>::SharedPtr subscription_robotflag_2_;

    //
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    geometry_msgs::msg::TransformStamped transformStamped_b1;
    geometry_msgs::msg::TransformStamped transformStamped_b2;
    geometry_msgs::msg::TransformStamped transformStamped_b3;
    geometry_msgs::msg::TransformStamped transformStamped_b4;
    geometry_msgs::msg::TransformStamped transformStamped_b5;
    geometry_msgs::msg::TransformStamped transformStamped_b6;
    geometry_msgs::msg::TransformStamped transformStamped_b7;
    geometry_msgs::msg::TransformStamped transformStamped_r1;
    geometry_msgs::msg::TransformStamped transformStamped_r2;
    geometry_msgs::msg::TransformStamped transformStamped_r3;
    geometry_msgs::msg::TransformStamped transformStamped_r4;
    geometry_msgs::msg::TransformStamped transformStamped_r5;
    geometry_msgs::msg::TransformStamped transformStamped_r6;
    geometry_msgs::msg::TransformStamped transformStamped_r7;

    geometry_msgs::msg::TransformStamped tf_camera_r;
    geometry_msgs::msg::TransformStamped tf_camera;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr send_tf_timer_;

    //
    int warn_flag[2][7] = {0};
    //

    // 全局参数

    rclcpp::Subscription<radar_interfaces::msg::GlobalParam>::SharedPtr subscription_param_;

    int moo = 0;
  };
}

#endif