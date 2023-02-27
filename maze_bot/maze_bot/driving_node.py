import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Driving(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        # msg.linear.x = 0.5
        msg.angular.z = 0.3
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    driving_publisher = Driving()
    rclpy.spin(driving_publisher)
    driving_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()