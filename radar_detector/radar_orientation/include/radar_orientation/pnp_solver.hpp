#include <iostream>
#include <opencv2/opencv.hpp>

#include "radar_interfaces/msg/calibration_tf.hpp"

namespace radar_orientation
{
    extern int status_flag;
    extern std::vector<std::vector<int>> point;
    class pnp_solver
    {
    public:
        void get_pnp_argument(std::vector<int64_t> param_0, int32_t region_num_param, std::vector<int64_t> region_list, std::vector<int64_t> region);
        void calibration_solver();
        void solver_2Dto3D();
        void solver_3Dto2D();

        std::vector<cv::Point2f> imagePoints;
        std::vector<cv::Point3f> worldPoints;
        // 3D点坐标
        double worldPoints_param[4][3];
        double region_param[19][3];

        std::vector<int64_t> Points4_list{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        std::vector<cv::Point3f> Points3d;

        // 2D图像中点坐标
        std::vector<cv::Point2f> Points2d;

        // 相机内参矩阵
        cv::Mat cameraMatrix;

        // 畸变系数
        cv::Mat distCoeffs;

        // 旋转向量
        cv::Mat rvec;

        // 平移向量
        cv::Mat tvec;

        rclcpp::Publisher<radar_interfaces::msg::CalibrationTf>::SharedPtr publisher_calibrationtf_;
        radar_interfaces::msg::CalibrationTf calibrationtf_message_;

        int region_pointnum = 0;
        int region_num = 0;
    };

}