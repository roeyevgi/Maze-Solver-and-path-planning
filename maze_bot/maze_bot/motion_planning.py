import cv2
import numpy as np
import math

show_cv2_imgs = True

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
        # Maze exit not reached?
        self.goal_not_reached_flag = True
        # Location of mini goals
        self.goal_pose_x = 0
        self.goal_pose_y = 0
        # Current mini goal iteration
        self.path_iter = 0


    def go_to_goal(self, robot_loc, path, velocity, velocity_publisher):
        # Calculate the angle and distance to the next goal.
        angle_to_goal, dist_to_goal = self.angle_and_dist(robot_loc, (self.goal_pose_x, self.goal_pose_y))
        angle_to_turn = angle_to_goal - self.robot_angle

        # Normalize the distance and angle. (100 ===> 1.5 , 0 ===> 0.2)
        speed = np.interp(dist_to_goal, [0, 100], [0.2, 1.5])
        angle = np.interp(angle_to_turn, [-360, 360], [-4, 4])
        
        print(f'angle to goal = {angle_to_goal}\n angle to turn = {angle_to_turn}\n simulation angle = {abs(angle)}')
        print(f'distance to goal = {dist_to_goal}')

        # If the robot is far away from the next goal, turn towards the goal.
        if dist_to_goal >= 2:
            velocity.angular.z = angle

        # Because differential drive has limitations, there is a need to adjust the robot speed with the amount of turn.
        # Large turn ==> less speed.
        if abs(angle) < 0.4:
            velocity.linear.x = speed
        elif abs(angle) < 0.8:
            velocity.linear.x = 0.02
        else:
            velocity.linear.x = 0.0
        
        # Publish the updated velocity.
        if self.goal_not_reached_flag or dist_to_goal <= 1:
            velocity_publisher.publish(velocity)
        # If the robot is within reasonable distance to the mini goal. (8 pixels)
        if dist_to_goal <= 8:
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            # If the final goal has not reached yet, stop moving.
            if self.goal_not_reached_flag:
                velocity_publisher.publish(velocity)
            # Reached final goal.
            if self.path_iter == len(path)-1:
                # First time?
                if self.goal_not_reached_flag:
                    self.goal_not_reached_flag = False
                    print('congratulations you solved the maze!!!!')
            else:
                # Navigate to the next mini goal
                self.path_iter += 1
                self.goal_pose_x = path[self.path_iter][0]
                self.goal_pose_y = path[self.path_iter][1]
                print(f'Current goal: ({self.goal_pose_x}, {self.goal_pose_y})')




    def navigate_path(self, robot_loc, path, velocity, velocity_publisher):
        # If a valid path found.
        if type(path) != int:
            # Try to reach the first mini goal.
            if self.path_iter == 0:
                self.goal_pose_x = path[self.path_iter][0]
                self.goal_pose_y = path[self.path_iter][1]

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

                print("\n\nCar angle (Image From Relation) = {} I-S Relation {} Car angle (Simulation) = {}".format(self.robot_angle,self.robot_angle_relation,
                                                                                                                    self.robot_sim_angle))
                print("Car angle_Initial (Image) = ",self.robot_angle)
                print("Car loc {}".format(robot_loc))

                # Go through the found path to reach the maze exit.
                self.go_to_goal(robot_loc, path, velocity, velocity_publisher)



        else:
            if not self.initial_point_extracted:
                self.initial_loc = robot_loc
                self.initial_point_extracted = True
            
            
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


    def bck_to_orig(self, pt,transform_arr,rot_mat):

        st_col = transform_arr[0] # cols X
        st_row = transform_arr[1] # rows Y
        tot_cols = transform_arr[2] # total_cols / width W
        tot_rows = transform_arr[3] # total_rows / height H
        
        # point --> (col(x),row(y)) XY-Convention For Rotation And Translated To MazeCrop (Origin)
        #pt_array = np.array( [pt[0]+st_col, pt[1]+st_row] )
        pt_array = np.array( [pt[0], pt[1]] )
        
        # Rot Matrix (For Normal XY Convention Around Z axis = [cos0 -sin0]) But for Image convention [ cos0 sin0]
        #                                                      [sin0  cos0]                           [-sin0 cos0]
        rot_center = (rot_mat @ pt_array.T).T# [x,y]
        
        # Translating Origin If neccasary (To get whole image)
        rot_cols = tot_cols#tot_rows
        rot_rows = tot_rows#tot_cols
        rot_center[0] = rot_center[0] + (rot_cols * (rot_center[0]<0) ) + st_col  
        rot_center[1] = rot_center[1] + (rot_rows * (rot_center[1]<0) ) + st_row 
        return rot_center
    

    def display_control_mechanism_in_action(self,bot_loc,path,img_shortest_path,bot_localizer,frame_disp):
        Doing_pt = 0
        Done_pt = 0

        path_i = self.path_iter
        
        # Circle to represent car current location
        img_shortest_path = cv2.circle(img_shortest_path, bot_loc, 3, (0,0,255))

        if ( (type(path)!=int) and ( path_i!=(len(path)-1) ) ):
            curr_goal = path[path_i]
            # Mini Goal Completed
            if path_i!=0:
                img_shortest_path = cv2.circle(img_shortest_path, path[path_i-1], 3, (0,255,0),2)
                Done_pt = path[path_i-1]
            # Mini Goal Completing   
            img_shortest_path = cv2.circle(img_shortest_path, curr_goal, 3, (0,140,255),2)
            Doing_pt = curr_goal
        else:
            # Only Display Final Goal completed
            img_shortest_path = cv2.circle(img_shortest_path, path[path_i], 10, (0,255,0))
            Done_pt = path[path_i]

        if Doing_pt!=0:
            Doing_pt = self.bck_to_orig(Doing_pt, bot_localizer.transform_arr, bot_localizer.rot_mat_rev)
            frame_disp = cv2.circle(frame_disp, (int(Doing_pt[0]),int(Doing_pt[1])), 3, (0,140,255),2)   
            #loc_car_ = self.bck_to_orig(loc_car, bot_localizer_obj.transform_arr, bot_localizer_obj.rot_mat_rev)
            #frame_disp = cv2.circle(frame_disp, (int(loc_car_[0]),int(loc_car_[1])), 3, (0,0,255))
         
            
        if Done_pt!=0:
            Done_pt = self.bck_to_orig(Done_pt, bot_localizer.transform_arr, bot_localizer.rot_mat_rev)
            if ( (type(path)!=int) and ( path_i!=(len(path)-1) ) ):
                pass
                #frame_disp = cv2.circle(frame_disp, (int(Done_pt[0]),int(Done_pt[1])) , 3, (0,255,0),2)   
            else:
                frame_disp = cv2.circle(frame_disp, (int(Done_pt[0]),int(Done_pt[1])) , 10, (0,255,0))  

        st = "len(path) = ( {} ) , path_iter = ( {} )".format(len(path),self.path_iter)        
        
        frame_disp = cv2.putText(frame_disp, st, (bot_localizer.orig_X+50,bot_localizer.orig_Y-30), cv2.FONT_HERSHEY_PLAIN, 1.2, (0,0,255))
        if show_cv2_imgs:
            cv2.imshow("maze (Shortest Path + Car Loc)",img_shortest_path)
        else:
            try:
                cv2.destroyWindow("maze (Shortest Path + Car Loc)")
            except:
                pass
