#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

#include "radar_interfaces/msg/status.hpp"
#include "radar_interfaces/msg/calibration_ui.hpp"
#include "radar_interfaces/msg/calibration_tf.hpp"

namespace calibration_ui
{
  class calibration_ui_node : public rclcpp::Node
  {
  public:
    explicit calibration_ui_node(const rclcpp::NodeOptions &options) : Node("publisher_node", options)
    {
      subscription_ = create_subscription<sensor_msgs::msg::Image>(
          "image_raw", 10, std::bind(&calibration_ui_node::videoCallback, this, std::placeholders::_1));

      subscription_calibrationui_ = create_subscription<radar_interfaces::msg::CalibrationUi>(
          "calibration", 10, std::bind(&calibration_ui_node::CalibrationCallback, this, std::placeholders::_1));

      subscription_calibrationtf_ = create_subscription<radar_interfaces::msg::CalibrationTf>(
          "calibration_tf", 10, std::bind(&calibration_ui_node::CalibrationTfCallback, this, std::placeholders::_1));

      image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("calibration_ui", 10);
    }

  private:
    void videoCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      // 转换ROS 2消息为OpenCV格式
      try
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      // 使用cv_ptr来访问OpenCV格式的视频流数据
      img = cv_ptr->image;
      image = img;
      // 标定框
      points.clear();
      
      points.push_back(cv::Point(point_x[0], point_y[0]));
      points.push_back(cv::Point(point_x[1], point_y[1]));
      points.push_back(cv::Point(point_x[2], point_y[2]));
      points.push_back(cv::Point(point_x[3], point_y[3]));

      cv::polylines(image, points, true, cv::Scalar(0, 255, 0), 2);
      // 区域1
      points.clear();
      points.push_back(cv::Point(region1[0][0], region1[0][1]));
      points.push_back(cv::Point(region1[1][0], region1[1][1]));
      points.push_back(cv::Point(region1[2][0], region1[2][1]));
      points.push_back(cv::Point(region1[3][0], region1[3][1]));
      points.push_back(points[0]);

      cv::polylines(image, points, true, cv::Scalar(255, 0, 0), 2);

      img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
      image_pub_->publish(*img_msg_); // 发布图像消息
    }

    void CalibrationCallback(const radar_interfaces::msg::CalibrationUi::SharedPtr msg)
    {
      point_select = msg->point;
      point_x[0] = msg->base_x1;
      point_y[0] = msg->base_y1;
      point_x[1] = msg->base_x2;
      point_y[1] = msg->base_y2;
      point_x[2] = msg->base_x3;
      point_y[2] = msg->base_y3;
      point_x[3] = msg->base_x4;
      point_y[3] = msg->base_y4;
    }

    void CalibrationTfCallback(const radar_interfaces::msg::CalibrationTf::SharedPtr msg)
    {
      region1[0][0] = (int)msg->region[0];
      region1[0][1] = (int)msg->region[1];
      region1[1][0] = (int)msg->region[2];
      region1[1][1] = (int)msg->region[3];
      region1[2][0] = (int)msg->region[4];
      region1[2][1] = (int)msg->region[5];
      region1[3][0] = (int)msg->region[6];
      region1[3][1] = (int)msg->region[7];
    }

    // 接收相机图像
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::Mat image;
    cv::Mat img;
    cv_bridge::CvImageConstPtr cv_ptr;

    // 接收ui信息

    rclcpp::Subscription<radar_interfaces::msg::CalibrationUi>::SharedPtr subscription_calibrationui_;

    // ui图像发布
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    sensor_msgs::msg::Image::SharedPtr img_msg_;

    // ui
    std::vector<cv::Point> points;

    rclcpp::Subscription<radar_interfaces::msg::CalibrationTf>::SharedPtr subscription_calibrationtf_;

    int point_select = 0;

    int point_x[4];
    int point_y[4];

    int region1[4][2];
  };
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(calibration_ui::calibration_ui_node);
