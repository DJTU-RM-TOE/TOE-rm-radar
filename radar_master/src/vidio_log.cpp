#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber(const rclcpp::NodeOptions &options) : Node("image_to_video_node", options)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw_1", rclcpp::SensorDataQoS(),
      std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));

    video_writer_ = nullptr;
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  { 
    cv_bridge::CvImagePtr cv_image;
    try
    {
      cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (video_writer_ == nullptr)
    {
      int height = cv_image->image.rows;
      int width = cv_image->image.cols;

      video_writer_ = std::make_unique<cv::VideoWriter>("/home/evence/output.mp4",
                                                        cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
                                                        30,
                                                        cv::Size(width, height));
    }

    video_writer_->write(cv_image->image);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::unique_ptr<cv::VideoWriter> video_writer_;
};

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ImageSubscriber);