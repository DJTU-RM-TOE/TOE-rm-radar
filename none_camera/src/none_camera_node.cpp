#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

namespace none_camera
{
  class NoneCameraNode : public rclcpp::Node
  {
  public:
    explicit NoneCameraNode(const rclcpp::NodeOptions &options)
        : Node("video_publisher", options)
    {
      // 创建一个标准视频流发布者
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(this->declare_parameter("topic_name", "image_raw"), 10);

      // 加载视频文件
      video_ = cv::VideoCapture("/home/evence/ros2_ws/toe_ctrl/src/vidio/00009.mp4");

      // 检查视频是否成功打开
      if (!video_.isOpened())
      {
        RCLCPP_ERROR(this->get_logger(), "无法打开视频文件");
      }

      // 设置发布频率，这里设置为每秒100帧
      timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&NoneCameraNode::publishFrame, this));
    }

  private:
    void publishFrame()
    {
      cv::Mat frame;
      // 读取视频帧
      video_ >> frame;

      // 检查视频是否结束，如果结束则重新播放
      if (frame.empty())
      {
        video_.set(cv::CAP_PROP_POS_FRAMES, 0);
        video_ >> frame;
      }

      // 将OpenCV的Mat转换为ROS2的图像消息
      auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
      image_msg->header.stamp = this->get_clock()->now();
      image_msg->height = frame.rows;
      image_msg->width = frame.cols;
      image_msg->encoding = "bgr8";
      image_msg->is_bigendian = false;
      image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      size_t size = frame.step * frame.rows;
      image_msg->data.resize(size);
      memcpy(&image_msg->data[0], frame.data, size);

      // 发布图像消息
      publisher_->publish(*image_msg);
    }

  private:
    cv::VideoCapture video_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
  };
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(none_camera::NoneCameraNode)
