import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
from .robot_localization import RobotLocalizer
from .robot_mapping import RobotMapper
from .path_planning import PathPlanner
import numpy as np

from geometry_msgs.msg import Twist


class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.subscriber = self.create_subscription(Image, '/top_camera/image_raw', self.get_video_feed_cb, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.maze_solving)
        self.bridge = CvBridge() # Convert the ros images to openCV data.
        self.robot_localizer = RobotLocalizer()
        self.robot_mapper = RobotMapper()
        self.robot_path_planner = PathPlanner()
        self.sat_view = np.zeros((100, 100))
        self.vel_msg = Twist()

    
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
        self.robot_path_planner.find_path_and_display(self.robot_mapper.graph.graph, start, end, maze, method='dijisktra')
        self.robot_path_planner.find_path_and_display(self.robot_mapper.graph.graph, start, end, maze, method='a_star')
        print(f'\nNode visited: Dijisktra VS A-Star = {self.robot_path_planner.dijisktra.dijisktra_node_visited} <----> {self.robot_path_planner.a_star.a_star_node_visited}')
        cv2.waitKey(0)


        # msg.linear.x = 0.5
        # msg.angular.z = 0.3
        self.publisher_.publish(self.vel_msg)



def main():
    rclpy.init()
    maze_solver = MazeSolver()
    rclpy.spin(maze_solver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
