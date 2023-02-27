import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import sys


class GoToGoal(Node):

    def __init__(self):
        super().__init__('go_to_goal_node')
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        timer_period = 0.2 # seconds
        self.timer = self.create_timer(timer_period, self.go_to_goal)
        self.robot_pose = Point()
        self.goal_position = Point()
        self.angle_offset = 0.0
        self.dist_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.angle_err = 0.0
        

    def pose_callback(self, data):
        # Get the robot position. (x,y)
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        # Converts the quaternion to euler angles. (in radians)
        (roll, pitch, yaw) = self.euler_from_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        # The corrent angle to the x axis.
        self.robot_pose.z = yaw


    def go_to_goal(self):
        # Get the required position from the terminal as goal position (x, y)
        self.goal_position.x = float(sys.argv[1])
        self.goal_position.y = float(sys.argv[2])
        self.angle_offset = float(sys.argv[3])    # To make sure the angle in between [-360, 360].

        vel_msg = Twist()  # cmd_vel msg.

        # Euclidean distance formula.        
        self.dist_to_goal = math.sqrt(pow((self.goal_position.x - self.robot_pose.x),2) + pow((self.goal_position.y - self.robot_pose.y),2)) 
        self.angle_to_goal = math.atan2(self.goal_position.y - self.robot_pose.y, self.goal_position.x - self.robot_pose.x) + self.angle_offset
        self.angle_err = self.angle_to_goal - self.robot_pose.z

        # Converge to the goal angle by an error of 0.1 radians.
        if abs(self.angle_err) > 0.1:
            # Don't move forward, only converge to the angle.
            vel_msg.linear.x = 0.0
            # Turn with angular velocity that is equal to the angle error.
            vel_msg.angular.z = self.angle_err
        else:
            # Move forward with velocity that is equal to the distance.
            vel_msg.linear.x = self.dist_to_goal
        

        msg = 'Distance to goal: {:3f}      Angle_to_goal: {:3f}'.format(self.dist_to_goal, self.angle_to_goal)
        self.get_logger().info(msg)
        self.velocity_pub.publish(vel_msg)  # Publish the velocity

    # Automatic Addison function that converts quaternions into euler angles.
    def euler_from_quaternion(self, x, y, z, w):
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians


def main():
    rclpy.init()

    go_to_goal_node = GoToGoal()

    rclpy.spin(go_to_goal_node)
    go_to_goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()