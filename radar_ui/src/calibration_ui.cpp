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
      // 全局参数
      parameters_client =
          std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_parameter_server");
      parameters_client->wait_for_service();

      // 参数
      region_list = this->declare_parameter<std::vector<int64_t>>("region_list");
      region_num = this->declare_parameter<int32_t>("region_num");
      for (int i = 0; i < region_num; i++)
      {
        region_pointnum += region_list[i];
      }

      RCLCPP_INFO(this->get_logger(), "%d", region_num);
      RCLCPP_INFO(this->get_logger(), "%d", region_pointnum);

      RCLCPP_INFO(this->get_logger(), "开始运行");
      subscription_1_ = create_subscription<sensor_msgs::msg::Image>(
          "image_raw_1", 10, std::bind(&calibration_ui_node::videoCallback_1, this, std::placeholders::_1));

      subscription_2_ = create_subscription<sensor_msgs::msg::Image>(
          "image_raw_2", 10, std::bind(&calibration_ui_node::videoCallback_2, this, std::placeholders::_1));

      subscription_calibrationui_ = create_subscription<radar_interfaces::msg::CalibrationUi>(
          "calibration_1", 10, std::bind(&calibration_ui_node::CalibrationCallback, this, std::placeholders::_1));

      subscription_calibrationtf_ = create_subscription<radar_interfaces::msg::CalibrationTf>(
          "calibration_tf_1", 10, std::bind(&calibration_ui_node::CalibrationTfCallback, this, std::placeholders::_1));

      image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("calibration_ui", 10);

      video_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&calibration_ui_node::videoCallback, this));
      parma_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&calibration_ui_node::parmaCallback, this));
    }

  private:
    void parmaCallback()
    {
      parameters_state = parameters_client->get_parameters(
          {"state"},
          std::bind(&calibration_ui_node::callbackGlobalParam, this, std::placeholders::_1));
    }
    void callbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future)
    {
      result = future.get();
      param = result.at(0);
      status_flag = param.as_int();
    }

    void videoCallback_1(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      // 转换ROS 2消息为OpenCV格式
      try
      {
        cv_ptr_1 = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      // 使用cv_ptr来访问OpenCV格式的视频流数据
      img_1 = cv_ptr_1->image;
      image_1 = img_1;
    }

    void videoCallback_2(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      // 转换ROS 2消息为OpenCV格式
      try
      {
        cv_ptr_2 = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      // 使用cv_ptr来访问OpenCV格式的视频流数据
      img_2 = cv_ptr_2->image;
      image_2 = img_2;
    }

    void videoCallback()
    {
      // 获取状态参数

      int width = image_1.cols;
      int height = image_1.rows;

      if (width == 0 && height == 0)
        return;

      // 标定框
      points.clear();

      points.push_back(cv::Point(point_x[0], point_y[0]));
      points.push_back(cv::Point(point_x[1], point_y[1]));
      points.push_back(cv::Point(point_x[2], point_y[2]));
      points.push_back(cv::Point(point_x[3], point_y[3]));

      cv::polylines(image_1, points, true, cv::Scalar(0, 255, 0), 1);

      int x_start = std::max(point_x[point_select] - 5, 0);
      int y_start = std::max(point_y[point_select] - 5, 0);
      int x_end = std::min(point_x[point_select] + 6, width - 1);
      int y_end = std::min(point_y[point_select] + 6, height - 1);

      cv::Rect range(x_start, y_start, x_end - x_start, y_end - y_start);

      cv::Mat roi_ = image_1(range);
      cv::Mat roi;

      cv::resize(roi_, roi, cv::Size(220, 220), 0, 0, cv::INTER_LINEAR);

      // 区域1

      // int leave = 0;
      // for (int i; i < 1; i++)
      //{
      //   points.clear();
      //   for (int j; j < region_list[i]; j++)
      //   {
      //     points.push_back(cv::Point(region[leave + j][0], region[leave + j][1]));
      //   }
      //
      //   cv::polylines(image, points, true, cv::Scalar(255, 0, 0), 2);
      //   leave += region_list[i];
      // }

      if (status_flag == 1)
      {

        points.clear();
        points.push_back(cv::Point(region[0][0], region[0][1]));
        points.push_back(cv::Point(region[1][0], region[1][1]));
        points.push_back(cv::Point(region[2][0], region[2][1]));
        points.push_back(cv::Point(region[3][0], region[3][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);

        points.clear();
        points.push_back(cv::Point(region[4][0], region[4][1]));
        points.push_back(cv::Point(region[5][0], region[5][1]));
        points.push_back(cv::Point(region[6][0], region[6][1]));
        points.push_back(cv::Point(region[7][0], region[7][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);

        points.clear();
        points.push_back(cv::Point(region[8][0], region[8][1]));
        points.push_back(cv::Point(region[9][0], region[9][1]));
        points.push_back(cv::Point(region[10][0], region[10][1]));
        points.push_back(cv::Point(region[11][0], region[11][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);

        points.clear();
        points.push_back(cv::Point(region[12][0], region[12][1]));
        points.push_back(cv::Point(region[13][0], region[13][1]));
        points.push_back(cv::Point(region[14][0], region[14][1]));
        points.push_back(cv::Point(region[15][0], region[15][1]));
        points.push_back(cv::Point(region[16][0], region[16][1]));
        points.push_back(cv::Point(region[17][0], region[17][1]));
        points.push_back(cv::Point(region[18][0], region[18][1]));

        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);
      }

      // 创建一个新的图像，用于合并两张图像
      cv::Mat merged_image(std::max(std::max(image_1.rows, image_2.rows), roi.rows), image_1.cols + image_2.cols + roi.cols, CV_8UC3);
      merged_image.setTo(cv::Scalar(0, 0, 0));

      // 将第一张图像复制到合并图像的左侧
      cv::Rect roi1(cv::Rect(0, 0, image_1.cols, image_1.rows));
      cv::Mat roi_image1(merged_image, roi1);
      image_1.copyTo(roi_image1);

      // 将第二张图像复制到合并图像的左侧
      cv::Rect roi2(cv::Rect(image_1.cols, 0, image_2.cols, image_2.rows));
      cv::Mat roi_image2(merged_image, roi2);
      image_2.copyTo(roi_image2);

      // 将第二张图像复制到合并图像的右侧
      cv::Rect roi3(cv::Rect(image_1.cols + image_2.cols, 0, roi.cols, roi.rows));
      cv::Mat roi_image3(merged_image, roi3);
      roi.copyTo(roi_image3);

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

    // 全局参数
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
    std::vector<rclcpp::Parameter> parameters;

    std::shared_future<std::vector<rclcpp::Parameter>> parameters_state;

    std::vector<rclcpp::Parameter> result;
    rclcpp::Parameter param;

    rclcpp::TimerBase::SharedPtr parma_timer_;

    int status_flag = 0;

    // 接收相机图像
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_1_;
    cv_bridge::CvImageConstPtr cv_ptr_1;
    cv::Mat image_1;
    cv::Mat img_1;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_2_;
    cv_bridge::CvImageConstPtr cv_ptr_2;
    cv::Mat image_2;
    cv::Mat img_2;

    // 接收ui信息

    rclcpp::Subscription<radar_interfaces::msg::CalibrationUi>::SharedPtr subscription_calibrationui_;

    // ui图像发布
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    sensor_msgs::msg::Image::SharedPtr img_msg_;

    // ui
    std::vector<cv::Point> points;

    rclcpp::Subscription<radar_interfaces::msg::CalibrationTf>::SharedPtr subscription_calibrationtf_;

    rclcpp::TimerBase::SharedPtr video_timer_;

    int point_select = 0;

    int point_x[4] = {0};
    int point_y[4] = {0};

    int region_num = 0;
    int region_pointnum = 0;
    int region[19][2] = {0};

    std::vector<int64_t> region_list;
  };

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(calibration_ui::calibration_ui_node);
