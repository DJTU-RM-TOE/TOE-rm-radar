#include <QApplication>
#include <QLabel>
#include <QPixmap>
#include <QThread>

class ImageThread : public QThread
{
public:
    void run() override
    {
        // 加载图像
        QPixmap pixmap("path_to_your_image.jpg");

        // 调整图像大小
        pixmap = pixmap.scaled(200, 200, Qt::KeepAspectRatio);

        // 发射图像准备好的信号
        emit imageReady(pixmap);
    }

signals:
    void imageReady(QPixmap pixmap);
};

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // 创建一个QLabel控件
    QLabel label;

    // 创建自定义线程对象
    ImageThread imageThread;

    // 当图像准备好时，更新标签上的图像
    QObject::connect(&imageThread, &ImageThread::imageReady, &label, [&label](QPixmap pixmap){
        label.setPixmap(pixmap);
    });

    // 启动线程
    imageThread.start();

    // 在标签上显示文本
    label.setText("Hello World");

    // 显示标签
    label.show();

    return a.exec();
}