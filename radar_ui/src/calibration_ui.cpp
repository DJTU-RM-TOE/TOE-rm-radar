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
    explicit calibration_ui_node(const rclcpp::NodeOptions &options) : Node("calibration_ui_node", options)
    {
      region_list = this->declare_parameter<std::vector<int64_t>>("region_list");
      region_num = this->declare_parameter<int32_t>("region_num");
      for (int i = 0; i < region_num; i++)
      {
        region_pointnum += region_list[i];
      }
      
      RCLCPP_INFO(this->get_logger(), "%d", region_num);
      RCLCPP_INFO(this->get_logger(), "%d", region_pointnum);

      RCLCPP_INFO(this->get_logger(), "开始运行");
      subscription_ = create_subscription<sensor_msgs::msg::Image>(
          "identification_image"/*"image_raw"*/, 10, std::bind(&calibration_ui_node::videoCallback, this, std::placeholders::_1));

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

      int width = image.cols;
      int height = image.rows;

      // 标定框
      points.clear();

      points.push_back(cv::Point(point_x[0], point_y[0]));
      points.push_back(cv::Point(point_x[1], point_y[1]));
      points.push_back(cv::Point(point_x[2], point_y[2]));
      points.push_back(cv::Point(point_x[3], point_y[3]));

      cv::polylines(image, points, true, cv::Scalar(0, 255, 0), 1);

      int x_start = std::max(point_x[point_select] - 5, 0);
      int y_start = std::max(point_y[point_select] - 5, 0);
      int x_end = std::min(point_x[point_select] + 6, width - 1);
      int y_end = std::min(point_y[point_select] + 6, height - 1);

      cv::Mat roi_ = image(cv::Range(y_start, y_end + 1), cv::Range(x_start, x_end + 1));
      cv::Mat roi;

      cv::resize(roi_, roi, cv::Size(220, 220), 0, 0, cv::INTER_LINEAR);

      // cv::Mat roi = image(cv::Range(point_x[0]-6, point_x[0]+5), cv::Range(point_y[0]-6, point_y[0]+5));

      // 区域1
      /*
      int leave = 0;
      for (int i; i < 1; i++)
      {
        points.clear();
        for (int j; j < region_list[i]; j++)
        {
          points.push_back(cv::Point(region[leave + j][0], region[leave + j][1]));
        }

        cv::polylines(image, points, true, cv::Scalar(255, 0, 0), 2);
        leave += region_list[i];
      }
      */
      points.clear();
      points.push_back(cv::Point(region[0][0], region[0][1]));
      points.push_back(cv::Point(region[1][0], region[1][1]));
      points.push_back(cv::Point(region[2][0], region[2][1]));
      points.push_back(cv::Point(region[3][0], region[3][1]));
      cv::polylines(image, points, true, cv::Scalar(255, 0, 0), 2);

      points.clear();
      points.push_back(cv::Point(region[4][0], region[4][1]));
      points.push_back(cv::Point(region[5][0], region[5][1]));
      points.push_back(cv::Point(region[6][0], region[6][1]));
      points.push_back(cv::Point(region[7][0], region[7][1]));
      cv::polylines(image, points, true, cv::Scalar(255, 0, 0), 2);

      points.clear();
      points.push_back(cv::Point(region[8][0], region[8][1]));
      points.push_back(cv::Point(region[9][0], region[9][1]));
      points.push_back(cv::Point(region[10][0], region[10][1]));
      points.push_back(cv::Point(region[11][0], region[11][1]));
      cv::polylines(image, points, true, cv::Scalar(255, 0, 0), 2);

      points.clear();
      points.push_back(cv::Point(region[12][0], region[12][1]));
      points.push_back(cv::Point(region[13][0], region[13][1]));
      points.push_back(cv::Point(region[14][0], region[14][1]));
      points.push_back(cv::Point(region[15][0], region[15][1]));
      points.push_back(cv::Point(region[16][0], region[16][1]));
      points.push_back(cv::Point(region[17][0], region[17][1]));
      points.push_back(cv::Point(region[18][0], region[18][1]));

      cv::polylines(image, points, true, cv::Scalar(255, 0, 0), 2);

      // 创建一个新的图像，用于合并两张图像
      cv::Mat merged_image(std::max(image.rows, roi.rows), image.cols + roi.cols, CV_8UC3);
      merged_image.setTo(cv::Scalar(0, 0, 0));

      // 将第一张图像复制到合并图像的左侧
      cv::Rect roi1(cv::Rect(0, 0, image.cols, image.rows));
      cv::Mat roi_image1(merged_image, roi1);
      image.copyTo(roi_image1);

      // 将第二张图像复制到合并图像的右侧
      cv::Rect roi2(cv::Rect(image.cols, 0, roi.cols, roi.rows));
      cv::Mat roi_image2(merged_image, roi2);
      roi.copyTo(roi_image2);

      img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", merged_image).toImageMsg();
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
      for (int i = 0; i < region_pointnum; i++)
      {
        region[i][0] = (int)msg->region[2 * i];
        region[i][1] = (int)msg->region[2 * i + 1];
      }
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

    int region_num = 0;
    int region_pointnum = 0;
    int region[19][2];

    std::vector<int64_t> region_list;
  };
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(calibration_ui::calibration_ui_node);
