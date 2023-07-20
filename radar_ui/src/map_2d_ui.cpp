#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
// msg
#include <radar_interfaces/msg/status.hpp>

// 场地实地大小数据 单位m
#define MAP_Y_MAX 28
#define MAP_X_MAX 15

#define MAP_CONSTANT 202

// 分辨率压缩率
#define SCALE_PERCENT 0.1

#define COLOR_B 0
#define COLOR_R 1

namespace map_2d_ui
{
    class Map2dUiNode : public rclcpp::Node
    {
    public:
        explicit Map2dUiNode(const rclcpp::NodeOptions &options) : Node("map_2d_ui_node", options)
        {

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // 读取图像
            Primitive_map = cv::imread("/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_ui/image/map_2D.jpg", cv::IMREAD_COLOR);

            image = Primitive_map;

            width = Primitive_map.cols;
            height = Primitive_map.rows;

            // 创建图像消息
            img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

            // 创建图像发布器
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("map2d_topic", 10);

            // 创建状态订阅器
            subscription_ = this->create_subscription<radar_interfaces::msg::Status>(
                "radar_status", 10, std::bind(&Map2dUiNode::topic_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "imgsize width %d height %d", width, height);

            // 定时发布图像消息
            timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]()
                                             {
                                                while(radar_status == 1)
                                                {
                                                    img_msg_->header.stamp = this->now(); // 更新时间戳

                                                    // 监听
                                                    Map2dUiNode::listenTf();
                                                    image = Primitive_map;

                                                    for (int j = 0; j < 6; j++)
                                                    {
                                                        cv::circle(image, Robot_point[COLOR_B][j], 60, cv::Scalar(255, 0, 0), -1);
                                                        cv::circle(image, Robot_point[COLOR_R][j], 60, cv::Scalar(0, 0, 255), -1);
                                                        putText(image, std::to_string(j), Robot_point[COLOR_B][j] + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                        putText(image, std::to_string(j), Robot_point[COLOR_R][j] + cv::Point(-30, 30), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 10);
                                                    }

                                                    cv::Size size(image.cols * SCALE_PERCENT, image.rows * SCALE_PERCENT);
                                                    cv::resize(image, map, size, 0, 0, cv::INTER_AREA);

                                                    img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", map).toImageMsg();
                                                    image_pub_->publish(*img_msg_); // 发布图像消息
                                                } });

            RCLCPP_INFO(this->get_logger(), "finish");
        }

    private:
        void listenTf()
        {
            try
            {
                transformStamped_num[COLOR_B][0] = tf_buffer_->lookupTransform("map", "RobotB1", tf2::TimePointZero);
                transformStamped_num[COLOR_B][1] = tf_buffer_->lookupTransform("map", "RobotB2", tf2::TimePointZero);
                transformStamped_num[COLOR_B][2] = tf_buffer_->lookupTransform("map", "RobotB3", tf2::TimePointZero);
                transformStamped_num[COLOR_B][3] = tf_buffer_->lookupTransform("map", "RobotB4", tf2::TimePointZero);
                transformStamped_num[COLOR_B][4] = tf_buffer_->lookupTransform("map", "RobotB5", tf2::TimePointZero);
                transformStamped_num[COLOR_B][5] = tf_buffer_->lookupTransform("map", "RobotB6", tf2::TimePointZero);
                transformStamped_num[COLOR_R][0] = tf_buffer_->lookupTransform("map", "RobotR1", tf2::TimePointZero);
                transformStamped_num[COLOR_R][1] = tf_buffer_->lookupTransform("map", "RobotR2", tf2::TimePointZero);
                transformStamped_num[COLOR_R][2] = tf_buffer_->lookupTransform("map", "RobotR3", tf2::TimePointZero);
                transformStamped_num[COLOR_R][3] = tf_buffer_->lookupTransform("map", "RobotR4", tf2::TimePointZero);
                transformStamped_num[COLOR_R][4] = tf_buffer_->lookupTransform("map", "RobotR5", tf2::TimePointZero);
                transformStamped_num[COLOR_R][5] = tf_buffer_->lookupTransform("map", "RobotR6", tf2::TimePointZero);

                for (int i = 0; i < 2; i++)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        Robot_point[i][j] = cv::Point(width / 2 + transformStamped_num[i][j].transform.translation.x * MAP_CONSTANT, height / 2 + transformStamped_num[i][j].transform.translation.y * MAP_CONSTANT);
                    }
                }
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to receive transform: %s", ex.what());
            }
        }

        void topic_callback(const radar_interfaces::msg::Status::SharedPtr msg)
        {
            radar_status = msg->status;
        }

        // 2dmap发布
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::Image::SharedPtr img_msg_;

        // tf监听
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // radar状态监听
        rclcpp::Subscription<radar_interfaces::msg::Status>::SharedPtr subscription_;

        // 坐标定义
        geometry_msgs::msg::TransformStamped transformStamped_num[2][6];
        cv::Point Robot_point[2][6];

        // 地图
        cv::Mat Primitive_map;
        cv::Mat map;

        // 带有标记的图像
        cv::Mat image;

        // 图像像素宽高
        int width;
        int height;

        // 状态
        int radar_status;
    };
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(map_2d_ui::Map2dUiNode)