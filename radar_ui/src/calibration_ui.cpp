#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

#include "radar_interfaces/msg/global_param.hpp"
#include "radar_interfaces/msg/calibration_ui.hpp"
#include "radar_interfaces/msg/calibration_tf.hpp"

namespace calibration_ui
{
  class calibration_ui_node : public rclcpp::Node
  {
  public:
    explicit calibration_ui_node(const rclcpp::NodeOptions &options) : Node("calibration_ui_node", options)
    {
      // 参数
      std::vector<int64_t> region_list_ = this->declare_parameter<std::vector<int64_t>>("region_list");
      region_num = this->declare_parameter<int32_t>("region_num");

      for (int i = 0; i < region_num; i++)
      {
        region_list[i] = static_cast<int>(region_list_[i]);
        region_pointnum += region_list_[i];
      }

      subscription_Param_ = this->create_subscription<radar_interfaces::msg::GlobalParam>(
          "global_param", 10,
          std::bind(&calibration_ui_node::paramCallback, this, std::placeholders::_1));

      subscription_ImageRaw_1_ = create_subscription<sensor_msgs::msg::Image>(
          "image_raw_1", rclcpp::SensorDataQoS(), std::bind(&calibration_ui_node::videoCallback_1, this, std::placeholders::_1));

      subscription_calibrationui_1_ = create_subscription<radar_interfaces::msg::CalibrationUi>(
          "calibration_1", 10, std::bind(&calibration_ui_node::CalibrationCallback1, this, std::placeholders::_1));

      subscription_calibrationtf_1_ = create_subscription<radar_interfaces::msg::CalibrationTf>(
          "calibration_tf_1", 10, std::bind(&calibration_ui_node::CalibrationTfCallback1, this, std::placeholders::_1));

      subscription_ImageRaw_2_ = create_subscription<sensor_msgs::msg::Image>(
          "image_raw_2", rclcpp::SensorDataQoS(), std::bind(&calibration_ui_node::videoCallback_2, this, std::placeholders::_1));

      subscription_calibrationui_2_ = create_subscription<radar_interfaces::msg::CalibrationUi>(
          "calibration_2", 10, std::bind(&calibration_ui_node::CalibrationCallback2, this, std::placeholders::_1));

      subscription_calibrationtf_2_ = create_subscription<radar_interfaces::msg::CalibrationTf>(
          "calibration_tf_2", 10, std::bind(&calibration_ui_node::CalibrationTfCallback2, this, std::placeholders::_1));

      image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("calibration_ui", 10);

      video_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&calibration_ui_node::videoCallback, this));
    }

  private:
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

      // 标定框
      points.clear();

      points.push_back(cv::Point(point_x[0], point_y[0]));
      points.push_back(cv::Point(point_x[1], point_y[1]));
      points.push_back(cv::Point(point_x[2], point_y[2]));
      points.push_back(cv::Point(point_x[3], point_y[3]));

      cv::polylines(image_1, points, true, cv::Scalar(0, 255, 0), 3);

      // 场地框
      if (status_flag == 1)
      {
        points.clear();
        points.push_back(cv::Point(region[0][0][0], region[0][0][1]));
        points.push_back(cv::Point(region[0][1][0], region[0][1][1]));
        points.push_back(cv::Point(region[0][2][0], region[0][2][1]));
        points.push_back(cv::Point(region[0][3][0], region[0][3][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);

        points.clear();
        points.push_back(cv::Point(region[0][4][0], region[0][4][1]));
        points.push_back(cv::Point(region[0][5][0], region[0][5][1]));
        points.push_back(cv::Point(region[0][6][0], region[0][6][1]));
        points.push_back(cv::Point(region[0][7][0], region[0][7][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);

        points.clear();
        points.push_back(cv::Point(region[0][8][0], region[0][8][1]));
        points.push_back(cv::Point(region[0][9][0], region[0][9][1]));
        points.push_back(cv::Point(region[0][10][0], region[0][10][1]));
        points.push_back(cv::Point(region[0][11][0], region[0][11][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);

        points.clear();
        points.push_back(cv::Point(region[0][12][0], region[0][12][1]));
        points.push_back(cv::Point(region[0][13][0], region[0][13][1]));
        points.push_back(cv::Point(region[0][14][0], region[0][14][1]));
        points.push_back(cv::Point(region[0][15][0], region[0][15][1]));
        points.push_back(cv::Point(region[0][16][0], region[0][16][1]));
        points.push_back(cv::Point(region[0][17][0], region[0][17][1]));
        points.push_back(cv::Point(region[0][18][0], region[0][18][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);

        points.clear();
        points.push_back(cv::Point(region[0][19][0], region[0][19][1]));
        points.push_back(cv::Point(region[0][20][0], region[0][20][1]));
        points.push_back(cv::Point(region[0][21][0], region[0][21][1]));
        points.push_back(cv::Point(region[0][22][0], region[0][22][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);

        points.clear();
        points.push_back(cv::Point(region[0][23][0], region[0][23][1]));
        points.push_back(cv::Point(region[0][24][0], region[0][24][1]));
        points.push_back(cv::Point(region[0][25][0], region[0][25][1]));
        points.push_back(cv::Point(region[0][26][0], region[0][26][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);

        points.clear();
        points.push_back(cv::Point(region[0][27][0], region[0][27][1]));
        points.push_back(cv::Point(region[0][28][0], region[0][28][1]));
        points.push_back(cv::Point(region[0][29][0], region[0][29][1]));
        points.push_back(cv::Point(region[0][30][0], region[0][30][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);

        points.clear();
        points.push_back(cv::Point(region[0][31][0], region[0][31][1]));
        points.push_back(cv::Point(region[0][32][0], region[0][32][1]));
        points.push_back(cv::Point(region[0][33][0], region[0][33][1]));
        points.push_back(cv::Point(region[0][34][0], region[0][34][1]));
        points.push_back(cv::Point(region[0][35][0], region[0][35][1]));
        points.push_back(cv::Point(region[0][36][0], region[0][36][1]));
        points.push_back(cv::Point(region[0][37][0], region[0][37][1]));
        cv::polylines(image_1, points, true, cv::Scalar(255, 0, 0), 2);
      }
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

      // 标定框
      points.clear();

      points.push_back(cv::Point(point_x[4], point_y[4]));
      points.push_back(cv::Point(point_x[5], point_y[5]));
      points.push_back(cv::Point(point_x[6], point_y[6]));
      points.push_back(cv::Point(point_x[7], point_y[7]));

      cv::polylines(image_2, points, true, cv::Scalar(0, 255, 0), 1);

      // 场地框

      int num = 0;
      if (status_flag == 1)
      {

        for (int i = 0; i < region_num; i++)
        {
          // points.clear();
          // for (int j = 0; j < region_list[region_num]; j++)
          //{
          //  // points.push_back(cv::Point(region[1][num+j][0], region[1][num+j][1]));
          //  RCLCPP_INFO(this->get_logger(), "region %d %d", region[1][num + j][0], region[1][num + j][1]);
          //}

          // cv::polylines(image_2, points, true, cv::Scalar(255, 0, 0), 2);
          // num += region_list[i];

          points.clear();
          points.push_back(cv::Point(region[1][0][0], region[1][0][1]));
          points.push_back(cv::Point(region[1][1][0], region[1][1][1]));
          points.push_back(cv::Point(region[1][2][0], region[1][2][1]));
          points.push_back(cv::Point(region[1][3][0], region[1][3][1]));
          cv::polylines(image_2, points, true, cv::Scalar(255, 0, 0), 2);

          points.clear();
          points.push_back(cv::Point(region[1][4][0], region[1][4][1]));
          points.push_back(cv::Point(region[1][5][0], region[1][5][1]));
          points.push_back(cv::Point(region[1][6][0], region[1][6][1]));
          points.push_back(cv::Point(region[1][7][0], region[1][7][1]));
          cv::polylines(image_2, points, true, cv::Scalar(255, 0, 0), 2);

          points.clear();
          points.push_back(cv::Point(region[1][8][0], region[1][8][1]));
          points.push_back(cv::Point(region[1][9][0], region[1][9][1]));
          points.push_back(cv::Point(region[1][10][0], region[1][10][1]));
          points.push_back(cv::Point(region[1][11][0], region[1][11][1]));
          cv::polylines(image_2, points, true, cv::Scalar(255, 0, 0), 2);

          points.clear();
          points.push_back(cv::Point(region[1][12][0], region[1][12][1]));
          points.push_back(cv::Point(region[1][13][0], region[1][13][1]));
          points.push_back(cv::Point(region[1][14][0], region[1][14][1]));
          points.push_back(cv::Point(region[1][15][0], region[1][15][1]));
          points.push_back(cv::Point(region[1][16][0], region[1][16][1]));
          points.push_back(cv::Point(region[1][17][0], region[1][17][1]));
          points.push_back(cv::Point(region[1][18][0], region[1][18][1]));
          cv::polylines(image_2, points, true, cv::Scalar(255, 0, 0), 2);

          points.clear();
          points.push_back(cv::Point(region[1][19][0], region[1][19][1]));
          points.push_back(cv::Point(region[1][20][0], region[1][20][1]));
          points.push_back(cv::Point(region[1][21][0], region[1][21][1]));
          points.push_back(cv::Point(region[1][22][0], region[1][22][1]));
          cv::polylines(image_2, points, true, cv::Scalar(255, 0, 0), 2);

          points.clear();
          points.push_back(cv::Point(region[1][23][0], region[1][23][1]));
          points.push_back(cv::Point(region[1][24][0], region[1][24][1]));
          points.push_back(cv::Point(region[1][25][0], region[1][25][1]));
          points.push_back(cv::Point(region[1][26][0], region[1][26][1]));
          cv::polylines(image_2, points, true, cv::Scalar(255, 0, 0), 2);

          points.clear();
          points.push_back(cv::Point(region[1][27][0], region[1][27][1]));
          points.push_back(cv::Point(region[1][28][0], region[1][28][1]));
          points.push_back(cv::Point(region[1][29][0], region[1][29][1]));
          points.push_back(cv::Point(region[1][30][0], region[1][30][1]));
          cv::polylines(image_2, points, true, cv::Scalar(255, 0, 0), 2);

          points.clear();
          points.push_back(cv::Point(region[1][31][0], region[1][31][1]));
          points.push_back(cv::Point(region[1][32][0], region[1][32][1]));
          points.push_back(cv::Point(region[1][33][0], region[1][33][1]));
          points.push_back(cv::Point(region[1][34][0], region[1][34][1]));
          points.push_back(cv::Point(region[1][35][0], region[1][35][1]));
          points.push_back(cv::Point(region[1][36][0], region[1][36][1]));
          points.push_back(cv::Point(region[1][37][0], region[1][37][1]));
          cv::polylines(image_2, points, true, cv::Scalar(255, 0, 0), 2);
        }
      }
    }

    void videoCallback()
    {
      // RCLCPP_INFO(this->get_logger(), "%d", region_num);
      // RCLCPP_INFO(this->get_logger(), "%d", region_pointnum);
      // RCLCPP_INFO(this->get_logger(), "region_list %d", region_list[0]);

      // 创建一个新的图像，用于合并两张图像
      cv::Mat merged_image(std::max(std::max(image_1.rows, image_2.rows), 220), image_1.cols + image_2.cols + 220, CV_8UC3);
      merged_image.setTo(cv::Scalar(0, 0, 0));

      int rolx = point_x[point_select] + (int)(point_select / 4) * image_1.cols;
      int roly = point_y[point_select];
      int x_start = std::max(rolx - 5, 0);
      int y_start = std::max(roly - 5, 0);
      int x_end = std::min(rolx + 6, merged_image.cols - 1);
      int y_end = std::min(roly + 6, merged_image.rows - 1);

      // RCLCPP_INFO(this->get_logger(), "x_start %d", x_start);
      // RCLCPP_INFO(this->get_logger(), "y_start %d", y_start);
      // RCLCPP_INFO(this->get_logger(), "x_end %d", x_end);
      // RCLCPP_INFO(this->get_logger(), "y_end %d", y_end);

      cv::Rect range(x_start, y_start, x_end - x_start, y_end - y_start);

      // 将第一张图像复制到合并图像的左侧
      cv::Rect roi1(cv::Rect(0, 0, image_1.cols, image_1.rows));
      cv::Mat roi_image1(merged_image, roi1);
      image_1.copyTo(roi_image1);

      // 将第二张图像复制到合并图像的左侧
      cv::Rect roi2(cv::Rect(image_1.cols, 0, image_2.cols, image_2.rows));
      cv::Mat roi_image2(merged_image, roi2);
      image_2.copyTo(roi_image2);

      cv::Mat roi_ = merged_image(range);
      cv::Mat roi;
      cv::resize(roi_, roi, cv::Size(220, 220), 0, 0, cv::INTER_LINEAR);

      // 将第三张图像复制到合并图像的右侧
      cv::Rect roi3(cv::Rect(image_1.cols + image_2.cols, 0, roi.cols, roi.rows));
      cv::Mat roi_image3(merged_image, roi3);
      roi.copyTo(roi_image3);

      cv::Mat final_image;
      cv::resize(merged_image, final_image, cv::Size(merged_image.cols / 4, merged_image.rows / 4), 0, 0, cv::INTER_LINEAR);

      img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", final_image).toImageMsg();

      image_pub_->publish(*img_msg_); // 发布图像消息
    }

    void CalibrationCallback1(const radar_interfaces::msg::CalibrationUi::SharedPtr msg)
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

    void CalibrationCallback2(const radar_interfaces::msg::CalibrationUi::SharedPtr msg)
    {
      point_select = msg->point;
      point_x[4] = msg->base_x1;
      point_y[4] = msg->base_y1;
      point_x[5] = msg->base_x2;
      point_y[5] = msg->base_y2;
      point_x[6] = msg->base_x3;
      point_y[6] = msg->base_y3;
      point_x[7] = msg->base_x4;
      point_y[7] = msg->base_y4;
    }

    void CalibrationTfCallback1(const radar_interfaces::msg::CalibrationTf::SharedPtr msg)
    {
      for (int i = 0; i < region_pointnum; i++)
      {
        region[0][i][0] = (int)msg->region[2 * i];
        region[0][i][1] = (int)msg->region[2 * i + 1];
      }
    }

    void CalibrationTfCallback2(const radar_interfaces::msg::CalibrationTf::SharedPtr msg)
    {
      for (int i = 0; i < region_pointnum; i++)
      {
        region[1][i][0] = (int)msg->region[2 * i];
        region[1][i][1] = (int)msg->region[2 * i + 1];
      }
    }

    void paramCallback(const radar_interfaces::msg::GlobalParam::SharedPtr msg)
    {
      status_flag = msg->status;
      color_flag = msg->color;
    }
    // 全局参数
    rclcpp::Subscription<radar_interfaces::msg::GlobalParam>::SharedPtr subscription_Param_;

    int status_flag = 0;
    int color_flag = 0;

    // 接收相机图像
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_ImageRaw_1_;
    cv_bridge::CvImageConstPtr cv_ptr_1;
    cv::Mat image_1;
    cv::Mat img_1;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_ImageRaw_2_;
    cv_bridge::CvImageConstPtr cv_ptr_2;
    cv::Mat image_2;
    cv::Mat img_2;

    // 接收ui信息

    rclcpp::Subscription<radar_interfaces::msg::CalibrationUi>::SharedPtr subscription_calibrationui_1_;
    rclcpp::Subscription<radar_interfaces::msg::CalibrationUi>::SharedPtr subscription_calibrationui_2_;

    // ui图像发布
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    sensor_msgs::msg::Image::SharedPtr img_msg_;

    // ui
    std::vector<cv::Point> points;

    rclcpp::Subscription<radar_interfaces::msg::CalibrationTf>::SharedPtr subscription_calibrationtf_1_;
    rclcpp::Subscription<radar_interfaces::msg::CalibrationTf>::SharedPtr subscription_calibrationtf_2_;

    rclcpp::TimerBase::SharedPtr video_timer_;

    int point_select = 0;

    int point_x[8] = {0};
    int point_y[8] = {0};

    int region_num = 0;
    int region_pointnum = 0;
    int region[2][38][2] = {0};

    int region_list[38] = {0};
  };

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(calibration_ui::calibration_ui_node);
