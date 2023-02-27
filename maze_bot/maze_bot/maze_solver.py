import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os

from geometry_msgs.msg import Twist


class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.subscriber = self.create_subscription(Image, '/top_camera/image_raw', self.get_video_feed_cb, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.maze_solving)
        self.bridge = CvBridge() # Convert the ros images to openCV data.

    
    def get_video_feed_cb(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.imshow('output', frame)
        cv2.waitKey(1)

    def maze_solving(self):
        msg = Twist()
        msg.linear.x = 0.5
        # msg.angular.z = 0.3
        self.publisher_.publish(msg)



def main():
    rclpy.init()
    maze_solver = MazeSolver()
    rclpy.spin(maze_solver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
