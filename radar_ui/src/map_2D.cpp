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

namespace map_2D_pub
{
    class map_2D_pub : public rclcpp::Node
    {
    public:
        map_2D_pub() : Node("map_2D_node")
        {

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // 读取图像
            map = cv::imread("/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_ui/image/map_2D.jpg", cv::IMREAD_COLOR);
            image = map;

            width = map.cols;
            height = map.rows;

            RCLCPP_INFO(this->get_logger(), "imgsize width %d height %d", width, height);

            // 创建图像消息
            img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

            // 创建图像发布器
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);

            // 定时发布图像消息
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]()
                                             {
                                                 img_msg_->header.stamp = this->now(); // 更新时间戳
                                                 img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                                                 image = map;
                                                 // Markers
                                                 map_2D_pub::listenTf();
                                                 cv::circle(image, cv::Point(1000, 1000), 60, cv::Scalar(0, 0, 255), -1);

                                                 image_pub_->publish(*img_msg_); // 发布图像消息
                                             });

            RCLCPP_INFO(this->get_logger(), "finish");
        }

    private:
        void listenTf()
        {
            try
            {
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped = tf_buffer_->lookupTransform("target_frame", "source_frame", tf2::TimePointZero);

                RCLCPP_INFO(this->get_logger(), "Received transform:");
                RCLCPP_INFO(this->get_logger(), "Translation: %f, %f, %f",
                            transformStamped.transform.translation.x,
                            transformStamped.transform.translation.y,
                            transformStamped.transform.translation.z);
                RCLCPP_INFO(this->get_logger(), "Rotation: %f, %f, %f, %f",
                            transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y,
                            transformStamped.transform.rotation.z,
                            transformStamped.transform.rotation.w);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to receive transform: %s", ex.what());
            }
        }

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::Image::SharedPtr img_msg_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // 地图
        cv::Mat map;
        // 带有标记的图像
        cv::Mat image;

        // 图像像素宽高
        int width;
        int height;
    };

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<map_2D_pub::map_2D_pub>());
    rclcpp::shutdown();

    return 0;
}
