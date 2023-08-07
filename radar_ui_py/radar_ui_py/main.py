import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time
import signal

running = True

# 定义信号处理函数
def signal_handler(signal, frame):
    global running
    running = False

# 注册信号处理函数
signal.signal(signal.SIGINT, signal_handler)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.subscription = self.create_subscription(
            Image, "image_topic", self.image_callback, 10
        )

    def image_callback(self, msg):
        try:
            global cv_image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            print('none img')
        # 在这里处理接收到的图像数据
        # 可以使用msg.data来访问图像数据


def ros2_node():
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    while running == False:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        print("线程1被中断")
        break
    
def qt_ui():
    while True:
        pass
        #try:
        #    #cv2.imshow("Image", cv_image)
        #    #cv2.waitKey(1)
        #    pass
        #    while running == False:
        #        break
        #except:
        #    pass
    
    
        
            

def main(args=None):
    thread1 = threading.Thread(target=ros2_node)
    thread2 = threading.Thread(target=qt_ui)
    
    thread1.daemon = True
    thread2.daemon = True

    # 启动线程
    thread1.start()
    thread2.start()

    # 等待线程执行完毕
    thread1.join()
    thread2.join()
    
    print("主线程结束")


if __name__ == "__main__":
    main()
