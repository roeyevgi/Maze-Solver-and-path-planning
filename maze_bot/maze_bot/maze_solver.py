import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
import os
from .robot_localization import RobotLocalizer
from .robot_mapping import RobotMapper
from .path_planning import PathPlanner
from .motion_planning import MotionPlanner
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.bridge = CvBridge() # Convert the ros images to openCV data.
        self.robot_localizer = RobotLocalizer()
        self.robot_mapper = RobotMapper()
        self.robot_path_planner = PathPlanner()
        self.robot_motion_planner = MotionPlanner()
        self.sat_view = np.zeros((100, 100))
        self.vel_msg = Twist()
        self.subscriber = self.create_subscription(Image, '/top_camera/image_raw', self.get_video_feed_cb, 10)
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.robot_motion_planner.get_pose, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.maze_solving)

    
    def get_video_feed_cb(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.sat_view = frame
        

    def maze_solving(self):
        frame_display = self.sat_view.copy()
        self.robot_localizer.localize_robot(self.sat_view, frame_display)
        self.robot_mapper.maze_to_graph(self.robot_localizer.maze_og)

        start = self.robot_mapper.graph.start
        end = self.robot_mapper.graph.end
        maze = self.robot_mapper.maze
        self.robot_path_planner.find_path_and_display(self.robot_mapper.graph.graph, start, end, maze, method='a_star')
        # cv2.waitKey(0)
        robot_loc = self.robot_localizer.car_location
        path = self.robot_path_planner.path_to_goal
        self.robot_motion_planner.navigate_path(robot_loc, path, self.vel_msg, self.velocity_publisher)


        # Setting the robot velocity
        # self.vel_msg.linear.x = 0.0
        # self.vel_msg.angular.z = 0.0

        self.velocity_publisher.publish(self.vel_msg)



def main():
    rclpy.init()
    maze_solver = MazeSolver()
    rclpy.spin(maze_solver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
