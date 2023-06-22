/*
#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
    // 3D世界中的点坐标
    std::vector<cv::Point3f> worldPoints;
    worldPoints.push_back(cv::Point3f(0, 0, 0));
    worldPoints.push_back(cv::Point3f(1, 0, 0));
    worldPoints.push_back(cv::Point3f(0, 1, 0));
    worldPoints.push_back(cv::Point3f(1, 1, 0));

    // 2D图像中��点坐标
    std::vector<cv::Point2f> imagePoints;
    imagePoints.push_back(cv::Point2f(10, 10));
    imagePoints.push_back(cv::Point2f(20, 10));
    imagePoints.push_back(cv::Point2f(10, 20));
    imagePoints.push_back(cv::Point2f(20, 20));

    // 相机内参矩阵
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);

    // 畸变系数
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0.1, 0.01, 0, 0, 0);

    // 旋转向量
    cv::Mat rvec;

    // 平移向量
    cv::Mat tvec;

    // PnP解算
    cv::solvePnP(worldPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    // 输出结果
    std::cout << "Rotation vector: " << rvec << std::endl;
    std::cout << "Translation vector: " << tvec << std::endl;

    return 0;
}
*/