#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <QApplication>
#include <QLabel>
#include <QImage>
#include <QPushButton>
#include <QVBoxLayout>

namespace qt5_ui
{
    class Qt5UiNode : public rclcpp::Node
    {
    public:
        explicit Qt5UiNode(const rclcpp::NodeOptions &options) : Node("map2d_topic", options)
        {
            RCLCPP_INFO(this->get_logger(), "Qt5UiNode开始运行");
            subscription_Map2dImage_ = create_subscription<sensor_msgs::msg::Image>("image_raw_1", rclcpp::SensorDataQoS(), std::bind(&Qt5UiNode::MapImage_Callback, this, std::placeholders::_1));
            
            int argc = 1;
            QApplication a(argc, nullptr);

            QWidget window;
            
            window.setWindowTitle("Image with Buttons");

            // 创建布局管理器
            QGridLayout layout;

            // 读取图像
            cv::Mat image = cv::imread("/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_ui/image/map_2D.jpg");

            cv::Mat resizedImage;
            cv::resize(image, resizedImage, cv::Size(), 0.1, 0.1);

            // 将OpenCV图像转换为Qt图像
            QImage qimage(resizedImage.data, resizedImage.cols, resizedImage.rows, QImage::Format_RGB888);

            // 创建标签并显示图像
            QLabel label;
            label.setPixmap(QPixmap::fromImage(qimage));
            label.setScaledContents(true);

            QPushButton button1("Button 1");
            QPushButton button2("Button 2");
            QPushButton button3("Button 3");
            QPushButton button4("Button 4");

            // 将标签和按钮添加到布局中
            layout.addWidget(&label, 0, 0);
            layout.addWidget(&button1, 1, 0);
            layout.addWidget(&button2, 2, 0);
            layout.addWidget(&button3, 0, 1);
            layout.addWidget(&button4, 1, 1);

            window.setLayout(&layout);

            Ui_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Qt5UiNode::Ui_Callback, this));

            window.show();
            a.exec();
        }

    private:
        void MapImage_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            try
            {
                Map2d_cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            Map2d_img = Map2d_cv_ptr->image;
        }
        
        void Ui_Callback()
        {
            RCLCPP_INFO(this->get_logger(), "循环");
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_Map2dImage_;
        cv_bridge::CvImageConstPtr Map2d_cv_ptr;
        cv::Mat Map2d_img;

        rclcpp::TimerBase::SharedPtr Ui_timer_;

        // QT
        
    };
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(qt5_ui::Qt5UiNode);

