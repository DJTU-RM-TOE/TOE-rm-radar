# ros2
import rclpy
from rclpy.node import Node

class RadarIdentificationSubscriber(Node):
    def __init__(self):
        print(1)
    
def main(args=None):
    rclpy.init(args=args)
    video_subscriber = RadarIdentificationSubscriber()
    rclpy.spin(video_subscriber)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()