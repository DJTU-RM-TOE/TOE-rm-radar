#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// 场地实地大小数据
#define MAP_Y_MAX 28
#define MAP_X_MAX 15

#define MAP_CONSTANT 202

// 分辨率压缩率
#define SCALE_PERCENT 0.1

namespace map_2D
{
    class map_2D_pub : public rclcpp::Node
    {
    public:
        map_2D_pub(const rclcpp::NodeOptions &options) : Node("map_2D_node")
        {

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // 读取图像
            Primitive_map = cv::imread("/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_ui/image/map_2D.jpg", cv::IMREAD_COLOR);

            image = Primitive_map;

            width = Primitive_map.cols;
            height = Primitive_map.rows;

            RCLCPP_INFO(this->get_logger(), "imgsize width %d height %d", width, height);

            // 创建图像消息
            img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

            // 创建图像发布器
            auto qos = rmw_qos_profile_sensor_data;
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);

            // 定时发布图像消息
            timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]()
                                             {
                                                 img_msg_->header.stamp = this->now(); // 更新时间戳
                                                 // Markers
                                                 map_2D_pub::listenTf();
                                                 image = Primitive_map;
                                                 cv::circle(image, RobotB1_point, 60, cv::Scalar(255, 0, 0), -1);
                                                 cv::circle(image, RobotB2_point, 60, cv::Scalar(255, 0, 0), -1);
                                                 cv::circle(image, RobotB3_point, 60, cv::Scalar(255, 0, 0), -1);
                                                 cv::circle(image, RobotB4_point, 60, cv::Scalar(255, 0, 0), -1);
                                                 cv::circle(image, RobotB5_point, 60, cv::Scalar(255, 0, 0), -1);
                                                 cv::circle(image, RobotB6_point, 60, cv::Scalar(255, 0, 0), -1);

                                                 cv::circle(image, RobotR1_point, 60, cv::Scalar(0, 0, 255), -1);
                                                 cv::circle(image, RobotR2_point, 60, cv::Scalar(0, 0, 255), -1);
                                                 cv::circle(image, RobotR3_point, 60, cv::Scalar(0, 0, 255), -1);
                                                 cv::circle(image, RobotR4_point, 60, cv::Scalar(0, 0, 255), -1);
                                                 cv::circle(image, RobotR5_point, 60, cv::Scalar(0, 0, 255), -1);
                                                 cv::circle(image, RobotR6_point, 60, cv::Scalar(0, 0, 255), -1);

                                                 putText(image, "1", RobotB1_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                 putText(image, "2", RobotB2_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                 putText(image, "3", RobotB3_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                 putText(image, "4", RobotB4_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                 putText(image, "5", RobotB5_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                 putText(image, "6", RobotB6_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);

                                                 putText(image, "1", RobotR1_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                 putText(image, "2", RobotR2_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                 putText(image, "3", RobotR3_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                 putText(image, "4", RobotR4_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                 putText(image, "5", RobotR5_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                 putText(image, "6", RobotR6_point + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);

                                                 cv::Size size(image.cols * SCALE_PERCENT, image.rows * SCALE_PERCENT);
                                                 cv::resize(image, map, size, 0, 0, cv::INTER_AREA);

                                                 img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", map).toImageMsg();
                                                 image_pub_->publish(*img_msg_); // 发布图像消息
                                             });

            RCLCPP_INFO(this->get_logger(), "finish");
        }

    private:
        void listenTf()
        {
            try
            {
                transformStamped_b1 = tf_buffer_->lookupTransform("map", "RobotB1", tf2::TimePointZero);
                transformStamped_b2 = tf_buffer_->lookupTransform("map", "RobotB2", tf2::TimePointZero);
                transformStamped_b3 = tf_buffer_->lookupTransform("map", "RobotB3", tf2::TimePointZero);
                transformStamped_b4 = tf_buffer_->lookupTransform("map", "RobotB4", tf2::TimePointZero);
                transformStamped_b5 = tf_buffer_->lookupTransform("map", "RobotB5", tf2::TimePointZero);
                transformStamped_b6 = tf_buffer_->lookupTransform("map", "RobotB6", tf2::TimePointZero);

                transformStamped_r1 = tf_buffer_->lookupTransform("map", "RobotR1", tf2::TimePointZero);
                transformStamped_r2 = tf_buffer_->lookupTransform("map", "RobotR2", tf2::TimePointZero);
                transformStamped_r3 = tf_buffer_->lookupTransform("map", "RobotR3", tf2::TimePointZero);
                transformStamped_r4 = tf_buffer_->lookupTransform("map", "RobotR4", tf2::TimePointZero);
                transformStamped_r5 = tf_buffer_->lookupTransform("map", "RobotR5", tf2::TimePointZero);
                transformStamped_r6 = tf_buffer_->lookupTransform("map", "RobotR6", tf2::TimePointZero);

                RobotB1_point = cv::Point(width / 2 + transformStamped_b1.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_b1.transform.translation.y * MAP_CONSTANT);
                RobotB2_point = cv::Point(width / 2 + transformStamped_b2.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_b2.transform.translation.y * MAP_CONSTANT);
                RobotB3_point = cv::Point(width / 2 + transformStamped_b3.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_b3.transform.translation.y * MAP_CONSTANT);
                RobotB4_point = cv::Point(width / 2 + transformStamped_b4.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_b4.transform.translation.y * MAP_CONSTANT);
                RobotB5_point = cv::Point(width / 2 + transformStamped_b5.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_b5.transform.translation.y * MAP_CONSTANT);
                RobotB6_point = cv::Point(width / 2 + transformStamped_b6.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_b6.transform.translation.y * MAP_CONSTANT);
                RobotR1_point = cv::Point(width / 2 + transformStamped_r1.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_r1.transform.translation.y * MAP_CONSTANT);
                RobotR2_point = cv::Point(width / 2 + transformStamped_r2.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_r2.transform.translation.y * MAP_CONSTANT);
                RobotR3_point = cv::Point(width / 2 + transformStamped_r3.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_r3.transform.translation.y * MAP_CONSTANT);
                RobotR4_point = cv::Point(width / 2 + transformStamped_r4.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_r4.transform.translation.y * MAP_CONSTANT);
                RobotR5_point = cv::Point(width / 2 + transformStamped_r5.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_r5.transform.translation.y * MAP_CONSTANT);
                RobotR6_point = cv::Point(width / 2 + transformStamped_r6.transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_r6.transform.translation.y * MAP_CONSTANT);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to receive transform: %s", ex.what());
            }
        }

        rclcpp::QoS qos = rclcpp::SensorDataQoS();

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::Image::SharedPtr img_msg_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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

        cv::Point RobotB1_point;
        cv::Point RobotB2_point;
        cv::Point RobotB3_point;
        cv::Point RobotB4_point;
        cv::Point RobotB5_point;
        cv::Point RobotB6_point;
        cv::Point RobotR1_point;
        cv::Point RobotR2_point;
        cv::Point RobotR3_point;
        cv::Point RobotR4_point;
        cv::Point RobotR5_point;
        cv::Point RobotR6_point;

        // 地图
        cv::Mat Primitive_map;
        cv::Mat map;
        // 带有标记的图像
        cv::Mat image;

        // 图像像素宽高
        int width;
        int height;
    };

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(map_2D::map_2D_pub)