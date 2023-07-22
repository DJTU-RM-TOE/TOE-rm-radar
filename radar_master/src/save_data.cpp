#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include "radar_interfaces/msg/status.hpp"
#include "radar_interfaces/msg/keyboard.hpp"
#include "radar_interfaces/msg/calibration_ui.hpp"
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace save_data
{
    class SaveDataNode : public rclcpp::Node
    {
    public:
        explicit SaveDataNode(const rclcpp::NodeOptions &options) : Node("SaveDataNode", options)
        {
            subscription_ = this->create_subscription<radar_interfaces::msg::Status>(
                "radar_status", 10, std::bind(&SaveDataNode::topic_callback, this, std::placeholders::_1));

            subscription_keyboard_ = create_subscription<radar_interfaces::msg::Keyboard>(
                "keyboard", 10, std::bind(&SaveDataNode::keyboardCallback, this, std::placeholders::_1));

            subscription_calibrationui_ = create_subscription<radar_interfaces::msg::CalibrationUi>(
                "calibration", 10, std::bind(&SaveDataNode::CalibrationCallback, this, std::placeholders::_1));
        }

    private:
        void keyboardCallback(const radar_interfaces::msg::Keyboard::SharedPtr msg)
        {
            if (msg->keynum == 10)
            {
                RCLCPP_INFO(this->get_logger(), "savedata!");
                savedata();
            }
        }

        void CalibrationCallback(const radar_interfaces::msg::CalibrationUi::SharedPtr msg)
        {
            if (radar_status != 0)
                return;
            point_select = msg->point;
            point_x[0] = msg->base_x1;
            point_y[0] = msg->base_y1;
            point_x[1] = msg->base_x2;
            point_y[1] = msg->base_y2;
            point_x[2] = msg->base_x3;
            point_y[2] = msg->base_y3;
            point_x[3] = msg->base_x4;
            point_y[3] = msg->base_y4;

            savedata();
        }

        void savedata()
        {

            // 写入完成
            cv::FileStorage tempFile("/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_master/config/ui_.yaml", cv::FileStorage::WRITE);

            int lineCount = 0;

            tempFile << "point" << point_select;
            tempFile << "base_x1" << point_x[0];
            tempFile << "base_y1" << point_y[0];
            tempFile << "base_x2" << point_x[1];
            tempFile << "base_y2" << point_y[1];
            tempFile << "base_x3" << point_x[2];
            tempFile << "base_y3" << point_y[2];
            tempFile << "base_x4" << point_x[3];
            tempFile << "base_y4" << point_y[3];

            tempFile.release();

            std::ofstream outputFile("/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_master/config/ui.yaml");
            std::ifstream tempFile_("/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_master/config/ui_.yaml");

            std::string line;
            while (std::getline(tempFile_, line))
            {
                if (line != "%YAML:1.0" && line != "---")
                {
                    outputFile<< line << std::endl;
                }
            }

            outputFile.close();
            tempFile_.close();

            RCLCPP_INFO(this->get_logger(), "success!");
        }

        void topic_callback(const radar_interfaces::msg::Status::SharedPtr msg)
        {
            radar_status = msg->status;
        }

        rclcpp::Subscription<radar_interfaces::msg::Keyboard>::SharedPtr subscription_keyboard_;
        rclcpp::Subscription<radar_interfaces::msg::CalibrationUi>::SharedPtr subscription_calibrationui_;

        // radar状态监听
        rclcpp::Subscription<radar_interfaces::msg::Status>::SharedPtr subscription_;
        int point_select = 0;

        int point_x[4];
        int point_y[4];

        int num = 0;

        // 状态
        int radar_status = 0;
    };
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(save_data::SaveDataNode);
