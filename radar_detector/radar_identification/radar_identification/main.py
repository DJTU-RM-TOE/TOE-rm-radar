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

from radar_interfaces.msg import RobotFlag


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
        
        self.publisher_ = self.create_publisher(RobotFlag, 'camera1', 10)
        
        self.publisher_img = self.create_publisher(Image, 'identification_image', 10)

    def callback(self, msg):
        
        final_boxes = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
        final_scores = [0,0,0,0,0,0,0,0,0,0,0,0]
        final_cls_inds = [0,0,0,0,0,0,0,0,0,0,0,0]

        robot_x = [0,0,0,0,0,0,0,0,0,0,0,0]
        robot_y = [0,0,0,0,0,0,0,0,0,0,0,0]
        robot_id = [0,0,0,0,0,0,0,0,0,0,0,0]
        
        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
        origin_img,final_boxes,final_scores,final_cls_inds = self.pred.inference(cv_image, conf=0.1, end2end=True)
        
        # 定义圆的颜色和厚度（-1表示填充）
        color = (0, 255, 0)
        thickness = -1
        radius = 5
        
        for i in range(12):
            robot_x[i] = 0
            robot_y[i] = 0
            robot_id[i] = 0
            if len(final_boxes) > i:
                robot_x[i] = int((final_boxes[i][0]+final_boxes[i][2])/2)
                robot_y[i] = int((final_boxes[i][1]+final_boxes[i][3])/2)
                robot_id[i] = final_cls_inds[i] + 1
                center = (robot_x[i], robot_y[i])
    
                cv2.circle(origin_img, center, radius, color, thickness)
            
        msg = RobotFlag()
        for i in range(12):
            msg.robot_2d[2*i] = robot_x[i]
            msg.robot_2d[2*i+1] = robot_y[i]
            msg.robot_id[i] = robot_id[i]
        self.publisher_.publish(msg)
        
        # 在OpenCV图像上进行处理
        # 例如，显示图像
        img_msg = self.cv_bridge.cv2_to_imgmsg(origin_img, encoding="bgr8")
        self.publisher_img.publish(img_msg)
        #cv2.imshow('Video', origin_img)
        
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = RadarIdentificationSubscriber()
    rclpy.spin(video_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

