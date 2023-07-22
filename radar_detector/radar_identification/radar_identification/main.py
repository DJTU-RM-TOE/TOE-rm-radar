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

from .utils.utils import preproc, vis
from .utils.utils import BaseEngine
import numpy as np
import time
import os
import argparse

final_boxes = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
final_scores = [0,0,0,0,0,0,0,0,0,0,0,0]
final_cls_inds = [0,0,0,0,0,0,0,0,0,0,0,0]

class Predictor(BaseEngine):
    def __init__(self, engine_path):
        super(Predictor, self).__init__(engine_path)
        self.n_classes = 3  # your model classes
        
class RadarIdentificationSubscriber(Node):
    def __init__(self):
        
        self.pred = Predictor(engine_path="/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_detector/radar_identification/radar_identification/model/yolov8n.trt")
        self.pred.get_fps()
        
        super().__init__('radar_identification')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.callback,
            10
        )
        self.cv_bridge = CvBridge()

    def callback(self, msg):
        
        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
        origin_img,final_boxes,final_scores,final_cls_inds = self.pred.inference(cv_image, conf=0.1, end2end=True)
        
        # 定义圆的颜色和厚度（-1表示填充）
        color = (0, 255, 0)
        thickness = -1
        radius = 5
            
        if len(final_boxes) > 0:
            x_0 = (final_boxes[0][0]+final_boxes[0][2])/2
            y_0 = (final_boxes[0][1]+final_boxes[0][3])/2
            center = (int(x_0), int(y_0))

            cv2.circle(origin_img, center, radius, color, thickness)
            
        if len(final_boxes) > 1:
            x_1 = (final_boxes[1][0]+final_boxes[1][2])/2
            y_1 = (final_boxes[1][1]+final_boxes[1][3])/2
            center = (int(x_1), int(y_1))
            cv2.circle(origin_img, center, radius, color, thickness)
        
        if len(final_boxes) > 2:
            x_2 = (final_boxes[2][0]+final_boxes[2][2])/2
            y_2 = (final_boxes[2][1]+final_boxes[2][3])/2
            center = (int(x_2), int(y_2))
            cv2.circle(origin_img, center, radius, color, thickness)
        
        # 在OpenCV图像上进行处理
        # 例如，显示图像
        cv2.imshow('Video', origin_img)
        
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = RadarIdentificationSubscriber()
    rclpy.spin(video_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

