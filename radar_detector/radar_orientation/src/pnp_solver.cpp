#include "rclcpp/rclcpp.hpp"
#include "radar_orientation/pnp_solver.hpp"

namespace radar_orientation
{
    void pnp_solver::calibration_solver()
    {
        
        worldPoints.push_back(cv::Point3f(2744, 5078, 1556));
        worldPoints.push_back(cv::Point3f(-5195, 1772, 615));
        worldPoints.push_back(cv::Point3f(-5195, 1112, 615));
        worldPoints.push_back(cv::Point3f(12166, 0, 1106));

        imagePoints.push_back(cv::Point2f(605, 553));
        imagePoints.push_back(cv::Point2f(875, 779));
        imagePoints.push_back(cv::Point2f(952, 779));
        imagePoints.push_back(cv::Point2f(924, 507));

        /*
        worldPoints.push_back(cv::Point3f(100, 100, ));
        worldPoints.push_back(cv::Point3f(-100, 100, ));
        worldPoints.push_back(cv::Point3f(-100, -100, 0));
        worldPoints.push_back(cv::Point3f(100, -100, 0));

        imagePoints.push_back(cv::Point2f(200, 400));
        imagePoints.push_back(cv::Point2f(400, 400));
        imagePoints.push_back(cv::Point2f(400, 600));
        imagePoints.push_back(cv::Point2f(200, 600));
        */

        cameraMatrix = (cv::Mat_<double>(3, 3) << 1821.366144, 0, 625.708393, 0, 1813.445553, 514.595196, 0, 0, 1);
        distCoeffs = (cv::Mat_<double>(5, 1) << -0.143932, 0.147997, 0, 0, 0);

        cv::solvePnP(worldPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, 5);
    }

    void pnp_solver::solver_2Dto3D()
    {
        cv::projectPoints(Points3d, rvec, tvec, cameraMatrix, distCoeffs, Points2d);
    }

    void pnp_solver::solver_3Dto2D()
    {
        Points3d.push_back(cv::Point3f(6000, 6000, 0));
        Points3d.push_back(cv::Point3f(-6000, 6000, 0));
        Points3d.push_back(cv::Point3f(-6000, -6000, 0));
        Points3d.push_back(cv::Point3f(6000, -6000, 0));
        cv::projectPoints(Points3d, rvec, tvec, cameraMatrix, distCoeffs, Points2d);
    }

}