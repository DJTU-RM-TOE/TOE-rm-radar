#ifndef __CALIBRATION__
#define __CALIBRATION__

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// 消息头文件
#include "radar_interfaces/msg/keyboard.hpp"
#include "radar_interfaces/msg/calibration_ui.hpp"

namespace radar_orientation
{
    extern int calibration_point[8][2];
    extern int point_select;
    extern int status_flag;
    class calibration
    {
    public:
        // 标定框点选取
        int point[4][2] = {0};
        int point_select = 0;

        // 标定框默认参数
        std::vector<int64_t> acquiesce{100, 100, 100, 200, 200, 200, 200, 100};

        // 载入默认参数
        void get_calibration_argument(std::vector<int64_t> param);
        void CalibrationUipub();

        // 键盘回调函数 同时发布标定框参数
        
        // 发布标定框
        rclcpp::Publisher<radar_interfaces::msg::CalibrationUi>::SharedPtr publisher_calibrationui_;

    private:
        int speed = 1;
    };
}

#endif
