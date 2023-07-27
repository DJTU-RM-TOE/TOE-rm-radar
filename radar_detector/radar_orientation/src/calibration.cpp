#include "radar_orientation/calibration.hpp"

#include "radar_interfaces/msg/keyboard.hpp"

namespace radar_orientation
{
    void calibration::get_calibration_argument(std::vector<int64_t> param)
    {
        point[0][0] = param[0];
        point[0][1] = param[1];
        point[1][0] = param[2];
        point[1][1] = param[3];
        point[2][0] = param[4];
        point[2][1] = param[5];
        point[3][0] = param[6];
        point[3][1] = param[7];
    }

    void calibration::keyboardCallback(const radar_interfaces::msg::Keyboard::SharedPtr msg)
    {
        int last_key[2] = {0};

        if (status_flag != 0)
            return;

        if (msg->keynum == 113 && point_select < 3)
            point_select++;
        else if (msg->keynum == 101 && point_select > 0)
            point_select--;
        else if (msg->keynum == 100 && point[point_select][0] < 1920)
            point[point_select][0] += speed;
        else if (msg->keynum == 97 && point[point_select][0] > 0)
            point[point_select][0] -= speed;
        else if (msg->keynum == 115 && point[point_select][1] < 1080)
            point[point_select][1] += speed;
        else if (msg->keynum == 119 && point[point_select][1] > 0)
            point[point_select][1] -= speed;
        else if (msg->keynum == 52 && msg->keynum < 3)
            speed ++;
        else if (msg->keynum == 46 && msg->keynum > 1)
            speed --;

        auto message = radar_interfaces::msg::CalibrationUi();
        message.point = point_select;
        message.base_x1 = point[0][0];
        message.base_y1 = point[0][1];
        message.base_x2 = point[1][0];
        message.base_y2 = point[1][1];
        message.base_x3 = point[2][0];
        message.base_y3 = point[2][1];
        message.base_x4 = point[3][0];
        message.base_y4 = point[3][1];

        publisher_calibrationui_->publish(message);
    }
}
