#include "radar_orientation/calibration.hpp"

#include "radar_interfaces/msg/keyboard.hpp"

namespace radar_orientation
{
    void calibration::get_calibration_argument(std::vector<int64_t> param)
    {
        for (int i = 0; i < 4; i++)
        {
            point[i][0] = param[i * 2];
            point[i][1] = param[i * 2 + 1];
        }
    }

    void calibration::CalibrationUipub()
    {
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
