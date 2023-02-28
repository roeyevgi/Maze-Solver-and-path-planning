import cv2
import numpy as np

def return_smallest_obj(contours, noise_th = 10):
    Min_Contour_Area = 1000
    Min_Contour_Idx = -1
    for idx, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area < Min_Contour_Area and area > noise_th:
            Min_Contour_Area = area
            Min_Contour_Idx = idx
            smallest_countour = True
    # print("min area: " + str(Min_Contour_Area))
    return Min_Contour_Idx

def return_largest_obj(img):
    contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
    Max_Contour_Area = 0
    Max_Contour_Idx = -1
    for idx, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area > Max_Contour_Area:
            Max_Contour_Area = area
            Max_Contour_Idx = idx
    largest_obj_img = np.zeros_like(img)
    if Max_Contour_Idx != -1:
        largest_obj_img = cv2.drawContours(largest_obj_img, contours, Max_Contour_Idx, 255, -1)
        largest_obj_img = cv2.drawContours(largest_obj_img, contours, Max_Contour_Idx, 255, 2)
    return largest_obj_img, contours[Max_Contour_Idx]
    