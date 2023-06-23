#include "rclcpp/rclcpp.hpp"
#include "radar_interfaces/msg/status.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

namespace calibration_ui
{
  class calibration_ui_node : public rclcpp::Node
  {
  public:
    explicit calibration_ui_node(const rclcpp::NodeOptions &options) : Node("publisher_node", options)
    {
      subscription_ = create_subscription<sensor_msgs::msg::Image>(
          "image_raw", 10, std::bind(&calibration_ui_node::videoCallback, this, std::placeholders::_1));

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
      image = cv_ptr->image;

      //标定框
      points.push_back(cv::Point(100, 100));
      points.push_back(cv::Point(100, 200));
      points.push_back(cv::Point(200, 200));
      points.push_back(cv::Point(200, 100));
      points.push_back(points[0]);

      cv::polylines(image, points, true, cv::Scalar(0, 255, 0), 2);

      img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
      image_pub_->publish(*img_msg_); // 发布图像消息
    }

    //接收相机图像
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::Mat image;
    cv_bridge::CvImageConstPtr cv_ptr;

    //ui图像发布
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    sensor_msgs::msg::Image::SharedPtr img_msg_;

    //ui
    std::vector<cv::Point> points;

  };
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(calibration_ui::calibration_ui_node);
