#include <QApplication>
#include <QLabel>
#include <QImage>
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // 读取图像
    cv::Mat image = cv::imread("/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_ui/image/map_2D.jpg");

    cv::Mat resizedImage;
    cv::resize(image, resizedImage, cv::Size(), 0.1, 0.1);

    // 将OpenCV图像转换为Qt图像
    QImage qimage(resizedImage.data, resizedImage.cols, resizedImage.rows, QImage::Format_RGB888);

    // 创建标签并显示图像
    QLabel label;
    label.setPixmap(QPixmap::fromImage(qimage));
    label.show();

    return a.exec();
}