#include "rclcpp/rclcpp.hpp"
#include "radar_orientation/pnp_solver.hpp"

namespace radar_orientation
{
    void pnp_solver::calibration_solver(int (&point)[4][2])
    {
        for (int i = 0; i < 4; i++)
        {
            worldPoints.push_back(cv::Point3f(worldPoints_param[i][0], worldPoints_param[i][1], worldPoints_param[i][2]));
            imagePoints.push_back(cv::Point2f(point[i][0], point[i][1]));
        }

        cameraMatrix = (cv::Mat_<double>(3, 3) << 1821.366144, 0.000000, 625.708393, 0.000000, 1813.445553, 514.595196, 0, 0, 1);
        distCoeffs = (cv::Mat_<double>(5, 1) << -0.3179, 0.9872, 0.0, 0.0, 0.000000);

        cv::solvePnP(worldPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, 5);
    }

    void pnp_solver::solver_2Dto3D()
    {
        cv::projectPoints(Points3d, rvec, tvec, cameraMatrix, distCoeffs, Points2d);
    }

    void pnp_solver::solver_3Dto2D()
    {
        for (int i = 0; i < region_pointnum; i++)
        {
            Points3d.push_back(cv::Point3f(region_param[i][0], region_param[i][1], region_param[i][2]));
        }
        cv::projectPoints(Points3d, rvec, tvec, cameraMatrix, distCoeffs, Points2d);
    }

    void pnp_solver::get_pnp_argument(std::vector<int64_t> param_0, int32_t region_num_param, std::vector<int64_t> region_list, std::vector<int64_t> region)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                worldPoints_param[i][j] = param_0[i * 3 + j];
            }
        }

        region_num = region_num_param;

        for (int i = 0; i < region_num; i++)
        {
            region_pointnum += region_list[i];
            region_list_num[i] = region_list[i];
        }

        for (int i = 0; i < region_pointnum; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                region_param[i][j] = region[i * 3 + j];
            }
        }
    }
}