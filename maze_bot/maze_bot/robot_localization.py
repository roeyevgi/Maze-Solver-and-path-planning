import cv2
import numpy as np
from .utilities import return_smallest_obj, return_largest_obj

show_cv2_imgs = False

class RobotLocalizer():
    def __init__(self):
        self.is_bg_extracted = False
        self.bg_model = []
        self.maze_og = []

        # Transformations (crop and rotate)
        self.orig_x = 0
        self.orig_y = 0
        self.orig_rows = 0
        self.orig_cols = 0
        self.transorm_arr = []
        self.orig_rot = 0
        self.rot_mat = 0
        self.car_location = 0



    def localize_robot(self, current_frame, frame_disp):
        if not self.is_bg_extracted:
            self.extract_bg(current_frame)
            self.is_bg_extracted = True
        # Foreground Extraction.
        change = cv2.absdiff(current_frame, self.bg_model)
        change_gray = cv2.cvtColor(change, cv2.COLOR_BGR2GRAY)
        # (src, threshold, max_value, type)
        change_mask = cv2.threshold(change_gray, 15, 255, cv2.THRESH_BINARY)[1]
        car_mask, car_contour = return_largest_obj(change_mask)
        self.get_car_location(car_contour, car_mask)
        # Drawing bounding circle around the car.
        center, radius = cv2.minEnclosingCircle(car_contour)
        car_circular_mask = cv2.circle(car_mask.copy(), (int(center[0]), int(center[1])), int(radius + radius*0.4), 255, 3)
        car_circular_mask = cv2.bitwise_xor(car_circular_mask, car_mask)
        frame_disp[car_mask>0] = frame_disp[car_mask>0] + (0, 64, 0)
        frame_disp[car_circular_mask>0] = (0, 0, 255)
        # Display Extracted car and localized car in frame.
        if show_cv2_imgs:
            cv2.imshow('Foreground car', car_mask)
            cv2.imshow('car_localized', frame_disp)
        # cv2.waitKey(1)


    # rois = region of intrest
    def return_rois_boundinghull(self, rois_mask, contours):
        maze_enclosure = np.zeros_like(rois_mask)
        if contours:
            contour = np.concatenate(contours)
            contour = np.array(contour)
            cv2.fillConvexPoly(maze_enclosure, contour, 255)
        largest_contour = cv2.findContours(maze_enclosure, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        hull = cv2.convexHull(largest_contour[0])
        cv2.drawContours(maze_enclosure, [hull], 0, 255)
        return hull


    def extract_bg(self, frame):
        # Extract the mask of all the region of intrest. (rois)
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray_img, 50, 150, None, 3)
        # (image, mode, method)[0] for the first output out of 2
        contours = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        rois_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

        for idx, _ in enumerate(contours):
            cv2.drawContours(rois_mask, contours, idx, 255, -1)
        # Remove the car from these rois. (the car is ths smallest object in the frame)
        min_contour_idx = return_smallest_obj(contours)
        rois_noCar_mask = rois_mask.copy()
        if min_contour_idx != -1:
            cv2.drawContours(rois_noCar_mask, contours, min_contour_idx, 0, -1)
            # Draw the car mask
            car_mask = np.zeros_like(rois_mask)
            cv2.drawContours(car_mask, contours, min_contour_idx, 255, -1)
            cv2.drawContours(car_mask, contours, min_contour_idx, 255, -1)
            cv2.drawContours(car_mask, contours, min_contour_idx, 255, 3)
            carLess_mask = cv2.bitwise_not(car_mask)
            frame_car_removed = cv2.bitwise_and(frame, frame, mask=carLess_mask)

            base_color = frame_car_removed[0][0]
            ground_replica = np.ones_like(frame) * base_color
            # Generating BG_model.
            self.bg_model = cv2.bitwise_and(ground_replica, ground_replica, mask=car_mask)
            self.bg_model = cv2.bitwise_or(self.bg_model, frame_car_removed)
        
        hull = self.return_rois_boundinghull(rois_mask, contours)
        [X, Y, W, H] = cv2.boundingRect(hull)
        maze = rois_noCar_mask[Y:Y+H, X:X+W]
        maze_occupencygrid = cv2.bitwise_not(maze)

        self.maze_og = cv2.rotate(maze_occupencygrid, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.update_frameOfReferance(X, Y, W, H, 90)
        if show_cv2_imgs:
            cv2.imshow("1a. rois_mask",rois_mask)
            cv2.imshow("1b. frame_car_remvd",frame_car_removed)
            cv2.imshow("1c. Ground_replica",ground_replica)
            cv2.imshow("1d. bg_model",self.bg_model)
            cv2.imshow("2. maze_og",self.maze_og)
            # cv2.waitKey(0)
            cv2.destroyAllWindows()


    def update_frameOfReferance(self, X, Y, W, H, angle):
        self.orig_x = X
        self.orig_y = Y
        self.orig_rows = H
        self.orig_cols = W
        self.orig_rot = angle
        self.transorm_arr = [X, Y, W, H]
        # Rotation matrix.
        self.rot_mat = np.array(
                                [
                                 [ np.cos(np.deg2rad(self.orig_rot)) , np.sin(np.deg2rad(self.orig_rot))],
                                 [-np.sin(np.deg2rad(self.orig_rot)) , np.cos(np.deg2rad(self.orig_rot))]
                                ]
                               )
        self.rot_mat_rev = np.array(
                                [
                                 [ np.cos(np.deg2rad(-self.orig_rot)) , np.sin(np.deg2rad(-self.orig_rot))],
                                 [-np.sin(np.deg2rad(-self.orig_rot)) , np.cos(np.deg2rad(-self.orig_rot))]
                                ]
                               )

    def get_centroid(self, contour):
        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cx, cy)

    def get_car_location(self, car_contour, car_mask):
        robot_center = self.get_centroid(car_contour)
        robot_center_arr = np.array(robot_center)
        robot_center_trans = np.zeros_like(robot_center_arr)
        robot_center_trans[0] = robot_center_arr[0] - self.orig_x
        robot_center_trans[1] = robot_center_arr[1] - self.orig_y
        robot_loc_on_maze = (self.rot_mat @ robot_center_trans.T).T
        # Translating the origin if neccesary
        rot_cols = self.orig_rows
        rot_rows = self.orig_cols
        robot_loc_on_maze[0] = robot_loc_on_maze[0] + (rot_cols * (robot_loc_on_maze[0]<0))
        robot_loc_on_maze[1] = robot_loc_on_maze[1] + (rot_cols * (robot_loc_on_maze[1]<0))

        self.car_location = (int(robot_loc_on_maze[0]), int(robot_loc_on_maze[1]))