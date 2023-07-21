'''
import rclpy
from rclpy.node import Node

from .utils.utils import preproc, vis
from .utils.utils import BaseEngine
import numpy as np
import cv2
import time
import os
import argparse


class Predictor(BaseEngine):
    def __init__(self, engine_path):
        super(Predictor, self).__init__(engine_path)
        self.n_classes = 3  # your model classes
        
class HelloWorldComponent(Node):
    def __init__(self):
        super().__init__('hello_world_component')

        pred = Predictor(engine_path="/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_detector/radar_identification/radar_identification/model/yolov8n.trt")
        pred.get_fps()
        video = "/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_detector/radar_identification/radar_identification/vidio/real.avi"

        pred.detect_video(video, conf=0.1, end2end=True) # set 0 use a webcam

def main(args=None):
    rclpy.init(args=args)

    hello_world_component = HelloWorldComponent()

    rclpy.spin(hello_world_component)

    hello_world_component.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RadarIdentificationSubscriber(Node):
    def __init__(self):
        super().__init__('radar_identification')
        self.subscription = self.create_subscription(
            Image,
            'video_topic',
            self.callback,
            10
        )
        self.cv_bridge = CvBridge()

    def callback(self, msg):
        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 在OpenCV图像上进行处理
        # 例如，显示图像
        cv2.imshow('Video', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = RadarIdentificationSubscriber()
    rclpy.spin(video_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

