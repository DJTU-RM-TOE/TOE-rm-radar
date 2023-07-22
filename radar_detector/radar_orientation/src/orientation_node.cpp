#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "radar_interfaces/msg/status.hpp"
#include "radar_interfaces/msg/keyboard.hpp"
#include "radar_interfaces/msg/calibration_ui.hpp"
#include "radar_interfaces/msg/robot_flag.hpp"

#include "radar_orientation/orientation_node.hpp"

namespace radar_orientation
{
  OrientationNode::OrientationNode(const rclcpp::NodeOptions &options) : Node("radar_orientation_node", options)
  {
    // 载入参数
    RCLCPP_INFO(this->get_logger(), "载入参数");
    calibration_module.get_calibration_argument(this->declare_parameter<std::vector<int64_t>>("base", calibration_module.acquiesce));
    pnp_solver_module.get_pnp_argument(this->declare_parameter<std::vector<int64_t>>("base_3d", pnp_solver_module.Points4_list),
                                       this->declare_parameter<int32_t>("region_num"),
                                       this->declare_parameter<std::vector<int64_t>>("region_list"),
                                       this->declare_parameter<std::vector<int64_t>>("region"));

    // 标定部分
    RCLCPP_INFO(this->get_logger(), "进入标定状态");
    subscription_keyboard_ = create_subscription<radar_interfaces::msg::Keyboard>(
        "keyboard", 10, std::bind(&calibration::keyboardCallback, &calibration_module, std::placeholders::_1));

    // 标定信息发布
    calibration_module.publisher_calibrationui_ = create_publisher<radar_interfaces::msg::CalibrationUi>("calibration", 10);

    // tf发布
    subscription_robotflag_ = create_subscription<radar_interfaces::msg::RobotFlag>(
        "camera1", 10, std::bind(&OrientationNode::send_tf, this, std::placeholders::_1));

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  void OrientationNode::detector_data()
  {
    transformStamped_b1.header.frame_id = "map";
    transformStamped_b1.child_frame_id = "RobotB1";
    transformStamped_b1.transform.translation.x = -1.0;
    transformStamped_b1.transform.translation.y = -6.0;
    transformStamped_b1.transform.translation.z = 0.0;

    transformStamped_b2.header.frame_id = "map";
    transformStamped_b2.child_frame_id = "RobotB2";
    transformStamped_b2.transform.translation.x = 0.0;
    transformStamped_b2.transform.translation.y = -5.0;
    transformStamped_b2.transform.translation.z = 0.0;

    transformStamped_b3.header.frame_id = "map";
    transformStamped_b3.child_frame_id = "RobotB3";
    transformStamped_b3.transform.translation.x = 0.0;
    transformStamped_b3.transform.translation.y = -4.0;
    transformStamped_b3.transform.translation.z = 0.0;

    transformStamped_b4.header.frame_id = "map";
    transformStamped_b4.child_frame_id = "RobotB4";
    transformStamped_b4.transform.translation.x = 0.0;
    transformStamped_b4.transform.translation.y = -3.0;
    transformStamped_b4.transform.translation.z = 0.0;

    transformStamped_b5.header.frame_id = "map";
    transformStamped_b5.child_frame_id = "RobotB5";
    transformStamped_b5.transform.translation.x = 0.0;
    transformStamped_b5.transform.translation.y = -2.0;
    transformStamped_b5.transform.translation.z = 0.0;

    transformStamped_b6.header.frame_id = "map";
    transformStamped_b6.child_frame_id = "RobotB6";
    transformStamped_b6.transform.translation.x = 0.0;
    transformStamped_b6.transform.translation.y = -1.0;
    transformStamped_b6.transform.translation.z = 0.0;

    transformStamped_r1.header.frame_id = "map";
    transformStamped_r1.child_frame_id = "RobotR1";
    transformStamped_r1.transform.translation.x = 0.0;
    transformStamped_r1.transform.translation.y = 1.0;
    transformStamped_r1.transform.translation.z = 0.0;

    transformStamped_r2.header.frame_id = "map";
    transformStamped_r2.child_frame_id = "RobotR2";
    transformStamped_r2.transform.translation.x = 0.0;
    transformStamped_r2.transform.translation.y = 2.0;
    transformStamped_r2.transform.translation.z = 0.0;

    transformStamped_r3.header.frame_id = "map";
    transformStamped_r3.child_frame_id = "RobotR3";
    transformStamped_r3.transform.translation.x = 0.0;
    transformStamped_r3.transform.translation.y = 3.0;
    transformStamped_r3.transform.translation.z = 0.0;

    transformStamped_r4.header.frame_id = "map";
    transformStamped_r4.child_frame_id = "RobotR4";
    transformStamped_r4.transform.translation.x = 0.0;
    transformStamped_r4.transform.translation.y = 4.0;
    transformStamped_r4.transform.translation.z = 0.0;

    transformStamped_r5.header.frame_id = "map";
    transformStamped_r5.child_frame_id = "RobotR5";
    transformStamped_r5.transform.translation.x = 0.0;
    transformStamped_r5.transform.translation.y = 5.0;
    transformStamped_r5.transform.translation.z = 0.0;

    transformStamped_r6.header.frame_id = "map";
    transformStamped_r6.child_frame_id = "RobotR6";
    transformStamped_r6.transform.translation.x = 0.0;
    transformStamped_r6.transform.translation.y = 6.0;
    transformStamped_r6.transform.translation.z = 0.0;
  }

  bool OrientationNode::pointInPolygon(cv::Point2f point, const std::vector<cv::Point2f> &polygon)
  {
    cv::Point2f pointInside(0, 0);
    int counter = 0;
    int i;
    double xinters;
    cv::Point2f p1, p2;

    int polygonSize = polygon.size();
    p1 = polygon[0];

    for (i = 1; i <= polygonSize; i++)
    {
      p2 = polygon[i % polygonSize];
      if (point.y > std::min(p1.y, p2.y))
      {
        if (point.y <= std::max(p1.y, p2.y))
        {
          if (point.x <= std::max(p1.x, p2.x))
          {
            if (p1.y != p2.y)
            {
              xinters = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
              if (p1.x == p2.x || point.x <= xinters)
                counter++;
            }
          }
        }
      }
      p1 = p2;
    }

    if (counter % 2 == 0)
      return false;
    else
      return true;
  }
  void OrientationNode::send_tf(const radar_interfaces::msg::RobotFlag::SharedPtr msg)
  {
    if (calibration_module.enter_flag == 1)
    {
      // 按下Enter正式执行主程序前信息发布的代码

      RCLCPP_INFO(this->get_logger(), "开始正式运行");
      status_flag = 1;
      publisher_status_ = this->create_publisher<radar_interfaces::msg::Status>("radar_status", 10);
      message_.status = status_flag;
      publisher_status_->publish(message_);
      calibration_module.enter_flag = 0;

      // 结算相机位姿态
      pnp_solver_module.calibration_solver();

      pnp_solver_module.solver_3Dto2D();

      // 发布
      pnp_solver_module.publisher_calibrationtf_ = this->create_publisher<radar_interfaces::msg::CalibrationTf>("calibration_tf", 10);
      pnp_solver_module.calibrationtf_message_.rvec = pnp_solver_module.rvec;
      pnp_solver_module.calibrationtf_message_.tvec = pnp_solver_module.tvec;

      RCLCPP_INFO(this->get_logger(), "%d", pnp_solver_module.region_pointnum);

      // 数组
      cv::Mat mat(pnp_solver_module.region_pointnum * 2, 1, CV_32S);

      // 逐个添加向量中的元素到矩阵中
      for (int i = 0; i < pnp_solver_module.region_pointnum; i++)
      {
        mat.at<cv::Vec2i>(2 * i, 0) = static_cast<int>(pnp_solver_module.Points2d[i].x);
        mat.at<cv::Vec2i>(2 * i + 1, 0) = static_cast<int>(pnp_solver_module.Points2d[i].y);
      }

      pnp_solver_module.calibrationtf_message_.region = mat;
      pnp_solver_module.publisher_calibrationtf_->publish(pnp_solver_module.calibrationtf_message_);

      double rx = pnp_solver_module.rvec.at<double>(0, 0);
      double ry = pnp_solver_module.rvec.at<double>(0, 1);
      double rz = pnp_solver_module.rvec.at<double>(0, 2);

      /*
      cv::Mat mat_rvec = cv::Mat::ones(3, 3, CV_64FC1);
      cv::Mat mat_tvec = cv::Mat::ones(1, 3, CV_64FC1);
      cv::Mat A = cv::Mat::ones(1, 3, CV_64FC1);

      mat_rvec.at<double>(0, 0) = 0;
      mat_rvec.at<double>(0, 1) = -rz;
      mat_rvec.at<double>(0, 2) = ry;
      mat_rvec.at<double>(1, 0) = rz;
      mat_rvec.at<double>(1, 1) = 0;
      mat_rvec.at<double>(1, 2) = -rx;
      mat_rvec.at<double>(2, 0) = -ry;
      mat_rvec.at<double>(2, 1) = rx;
      mat_rvec.at<double>(2, 2) = 0;

      mat_tvec.at<double>(0, 0) = pnp_solver_module.tvec.at<double>(0, 0);
      mat_tvec.at<double>(0, 1) = pnp_solver_module.tvec.at<double>(0, 1);
      mat_tvec.at<double>(0, 2) = pnp_solver_module.tvec.at<double>(0, 2);

      cv::Mat B = mat_tvec * mat_rvec * A;
      */

      tf_camera_r.header.frame_id = "map";
      tf_camera_r.child_frame_id = "camera_r";

      tf2::Quaternion q;
      q.setRPY(rx, ry, rz);

      tf_camera.header.frame_id = "camera_r";
      tf_camera.child_frame_id = "camera";

      tf_camera.transform.rotation.x = q.x();
      tf_camera.transform.rotation.y = q.y();
      tf_camera.transform.rotation.z = q.z();
      tf_camera.transform.rotation.w = q.w();

      tf_camera.transform.translation.x = pnp_solver_module.tvec.at<double>(0, 0) / 1000;
      tf_camera.transform.translation.y = pnp_solver_module.tvec.at<double>(0, 1) / 1000;
      tf_camera.transform.translation.z = pnp_solver_module.tvec.at<double>(0, 2) / 1000;

      // Send the transformation
    }

    detector_data();

    if (status_flag == 1)
    {
      //基于2d图像的警戒方案
      for (int i = 0; i < pnp_solver_module.region_num; i++)
      {
        warn_flag[i] = 0;
      }

      for (int i = 0; i < 12; i++)
      {
        cv::Point2f point(msg->robot_2d[2 * i], msg->robot_2d[2 * i + 1]);
        if (msg->robot_2d[2 * i] == 0 && msg->robot_2d[2 * i + 1] == 0)
          continue;
        int add = 0;
        for (int j = 0; j < pnp_solver_module.region_num; j++)
        {
          std::vector<cv::Point2f> firstPoints;
          firstPoints.resize(pnp_solver_module.region_list_num[j]);

          std::copy(pnp_solver_module.Points2d.begin() + add, pnp_solver_module.Points2d.begin() + pnp_solver_module.region_list_num[j], firstPoints.begin());
          add += pnp_solver_module.region_list_num[j];
          RCLCPP_INFO(this->get_logger(), "参数 %d %d ", add, pnp_solver_module.region_list_num[j]);
          RCLCPP_INFO(this->get_logger(), "框位 %d", j);
          RCLCPP_INFO(this->get_logger(), "点位 %d %d ", msg->robot_2d[2 * i], msg->robot_2d[2 * i + 1]);
          if (pointInPolygon(point, firstPoints))
          {
            RCLCPP_INFO(this->get_logger(), "进入警戒");
            warn_flag[j]++;
          }
        }
      }
      RCLCPP_INFO(this->get_logger(), "是否在框内 %d  %d  %d  %d", warn_flag[0], warn_flag[1], warn_flag[2], warn_flag[3]);

      /*
      std::vector<cv::Point2f> polygon;
      polygon.push_back(cv::Point2f(50, 50));
      polygon.push_back(cv::Point2f(100, 50));
      polygon.push_back(cv::Point2f(100, 100));
      polygon.push_back(cv::Point2f(50, 100));

      cv::Point2f point(75, 75);
      RCLCPP_INFO(this->get_logger(), "是否在框内 %d", pointInPolygon(point, polygon));
      */

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

      tf_camera_r.header.stamp = now();
      broadcaster_->sendTransform(tf_camera_r);

      tf_camera.header.stamp = now();
      broadcaster_->sendTransform(tf_camera);
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(radar_orientation::OrientationNode);
