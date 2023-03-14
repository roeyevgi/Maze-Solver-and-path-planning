import cv2
import numpy as np
import math

class MotionPlanner():
    def __init__(self):
        # Counter to move the car for a few iterations.
        self.count = 0
        # Is the initioal point extracted?
        self.initial_point_extracted = False
        # Initial car location.
        self.initial_loc = 0
        # Is the angle relation computed?
        self.angle_relation_computed = False

        # The angle from the image.
        self.robot_angle = 0
        # The angle from the simulation.
        self.robot_sim_angle = 0
        # Relation between the image and simulation angles.
        self.robot_angle_relation = 0

    def navigate_path(self, robot_loc, path, velocity, velocity_publisher):
        if self.count > 20:
            if not self.angle_relation_computed:
                # Stop the car.
                velocity.linear.x = 0.0
                velocity_publisher.publish(velocity)
                self.robot_angle, _ = self.angle_and_dist(self.initial_loc, robot_loc)
                self.robot_angle_relation = self.robot_sim_angle - self.robot_angle
                self.angle_relation_computed = True
            else:
                # If the relation is already computed, we know the robot image angle based on the relation and the simulation angle.
                self.robot_angle = self.robot_sim_angle - self.robot_angle_relation


        else:
            if not self.initial_point_extracted:
                self.initial_loc = robot_loc
                self.initial_point_extracted = True
            print("\n\nCar angle (Image From Relation) = {} I-S Relation {} Car angle (Simulation) = {}".format(self.robot_angle,self.robot_angle_relation,self.robot_sim_angle))
            print("Car angle_Initial (Image) = ",self.robot_angle)
            print("Car loc {}".format(robot_loc))
            
            velocity.linear.x = 1.0
            velocity_publisher.publish(velocity)

            self.count += 1

    
    def angle_and_dist(self, point_a, point_b):
        # Note that the Y axis of the image is in the opposite direction of the Y axis of the simulation.
        error_x = point_b[0] - point_a[0]
        error_y = point_a[1] - point_b[1]

        dist = math.sqrt(pow(error_x, 2) + pow(error_y, 2))

        angle = math.atan2(error_y, error_x)
        angle_deg = math.degrees(angle)
        
        # To get an angle in range [0, 360]
        if angle_deg > 0:
            return angle_deg, dist
        else:
            return angle_deg+360, dist
        

    def get_pose(self, data):
        quaternion = data.pose.pose.orientation
        # Converts the quaternion to euler angles. (in radians)
        (roll, pitch, yaw) = self.euler_from_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        # The corrent angle to the x axis.
        yaw_deg = math.degrees(yaw)

        # To get an angle in range [0, 360]
        if yaw > 0:
            self.robot_sim_angle = yaw_deg
        else:
            self.robot_sim_angle = yaw_deg + 360

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
