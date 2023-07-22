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

  // 标定框2D坐标
  // 标定框参数初始化
  std::vector<std::vector<int>> point{{0, 0},
                                        {0, 0},
                                        {0, 0},
                                        {0, 0}};

  // 主节点类
  class OrientationNode : public rclcpp::Node
  {
  public:
    OrientationNode(const rclcpp::NodeOptions &options);

  private:
    // 标定类
    calibration calibration_module;
    // pnp
    pnp_solver pnp_solver_module;

    double rvecArray[3];
    double tvecArray[3];

    void detector_data();
    void send_tf(const radar_interfaces::msg::RobotFlag::SharedPtr msg);
    bool pointInPolygon(cv::Point2f point, const std::vector<cv::Point2f>& polygon);

    rclcpp::Subscription<radar_interfaces::msg::Keyboard>::SharedPtr subscription_keyboard_;

    rclcpp::Subscription<radar_interfaces::msg::RobotFlag>::SharedPtr subscription_robotflag_;

    // 状态机发布
    rclcpp::Publisher<radar_interfaces::msg::Status>::SharedPtr publisher_status_;

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

    geometry_msgs::msg::TransformStamped tf_camera_r;
    geometry_msgs::msg::TransformStamped tf_camera;

    rclcpp::TimerBase::SharedPtr timer_;

    //
    int warn_flag[4];
    //
  };
}

#endif