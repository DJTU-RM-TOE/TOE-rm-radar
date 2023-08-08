#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <QApplication>
#include <QLabel>
#include <QImage>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <mutex>
#include <thread>

class Thread2Task
{
    public:
    void Handle()
    {
        cv::Mat image = GetImage();
        if (image.empty())
        {
            return;
        }
        // handle
        // 将OpenCV图像转换为Qt5格式的图像
        QImage qimage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);

        // 在Qt5中显示图像
        label_.setPixmap(QPixmap::fromImage(qimage));
        label_.show();
    }

    void UpdateImage(cv::Mat&& image)
    {
        std::lock_guard<std::mutex> lg(mtx_);
        image_ = std::move(image);
    }

    cv::Mat GetImage()
    {
        std::lock_guard<std::mutex> lg(mtx_);
        return image_;
    }
    private:
    std::mutex mtx_;
    cv::Mat image_;
    QLabel label_;
};

class ImageViewer : public rclcpp::Node
{
public:
  ImageViewer(Thread2Task& task2_handle)
  : Node("image_viewer"), task2_handle_(task2_handle)
  {
    // 创建一个ROS2订阅者，用于接收图像消息
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {

        // 将ROS2图像消息转换为OpenCV格式的图像
        cv::Mat image(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
        task2_handle_.UpdateImage(std::move(image));

        // 将OpenCV图像转换为Qt5格式的图像
        //QImage qimage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);

        // 在Qt5中显示图像
        //label_.setPixmap(QPixmap::fromImage(qimage));
        //label_.show();

      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  
Thread2Task& task2_handle_;
};



int main(int argc, char* argv[])
{
  // 初始化ROS2节点
  rclcpp::init(argc, argv);

  // 初始化Qt5应用程序
  QApplication app(argc, argv);

  Thread2Task task2;

  // 创建图像查看器对象
  ImageViewer image_viewer(task2);

  std::thread th([&task2](){
    while(1)
    {
        task2.Handle();
    }
  });
  // 运行Qt5应用程序
  return app.exec();
}