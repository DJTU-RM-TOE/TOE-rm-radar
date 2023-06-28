#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "radar_interfaces/msg/status.hpp"
#include "radar_interfaces/msg/keyboard.hpp"
#include "radar_interfaces/msg/calibration_ui.hpp"

#include "radar_orientation/orientation_node.hpp"

namespace radar_orientation
{
  OrientationNode::OrientationNode(const rclcpp::NodeOptions &options) : Node("radar_orientation_node", options)
  {
    // 载入参数
    RCLCPP_INFO(this->get_logger(), "载入参数");
    calibration_module.get_calibration_argument(this->declare_parameter<std::vector<int64_t>>("base", calibration_module.acquiesce));
    pnp_solver_module.get_pnp_argument(this->declare_parameter<std::vector<int64_t>>("base_3d", pnp_solver_module.Points4_list),
                                       this->declare_parameter<std::vector<int64_t>>("region_1", pnp_solver_module.Points4_list));

    // 标定部分
    RCLCPP_INFO(this->get_logger(), "进入标定状态");
    subscription_keyboard_ = create_subscription<radar_interfaces::msg::Keyboard>(
        "keyboard", 10, std::bind(&calibration::keyboardCallback, &calibration_module, std::placeholders::_1));

    // 标定信息发布
    calibration_module.publisher_calibrationui_ = create_publisher<radar_interfaces::msg::CalibrationUi>("calibration", 10);

    // tf发布
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    detector_data();
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&OrientationNode::send_tf, this));
  }

  void OrientationNode::detector_data()
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

  void OrientationNode::send_tf()
  {
    if (calibration_module.enter_flag == 1)
    {
      //按下Enter正式执行主程序前信息发布的代码

      RCLCPP_INFO(this->get_logger(), "开始正式运行");
      status_flag = 1;
      publisher_status_ = this->create_publisher<radar_interfaces::msg::Status>("radar_status", 10);
      message_.status = status_flag;
      publisher_status_->publish(message_);
      calibration_module.enter_flag = 0;

      // 结算相机位姿态
      pnp_solver_module.calibration_solver();

      pnp_solver_module.solver_3Dto2D();

      //发布
      pnp_solver_module.publisher_calibrationtf_ = this->create_publisher<radar_interfaces::msg::CalibrationTf>("calibration_tf", 10);
      pnp_solver_module.calibrationtf_message_.rvec = pnp_solver_module.rvec;
      pnp_solver_module.calibrationtf_message_.tvec = pnp_solver_module.tvec;

      // 数组
      cv::Mat mat(8, 1, CV_32S);

      // 逐个添加向量中的元素到矩阵中
      for (int i = 0; i < 4; i++)
      {
        mat.at<cv::Vec2i>(2*i, 0) = static_cast<int>(pnp_solver_module.Points2d[i].x);
        mat.at<cv::Vec2i>(2*i+1, 0) = static_cast<int>(pnp_solver_module.Points2d[i].y);
      }

      pnp_solver_module.calibrationtf_message_.region = mat;
      pnp_solver_module.publisher_calibrationtf_->publish(pnp_solver_module.calibrationtf_message_);
    }

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
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(radar_orientation::OrientationNode);

