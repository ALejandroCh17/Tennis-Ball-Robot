#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/robot2/robot-hat/TurboPi')
import cv2
import time
import math
import Camera
import numpy as np
import yaml_handle
from robot_hat import Motors

# Initialize camera and motors
camera = Camera.Camera()
motors = Motors()

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    areaMaxContour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:
                areaMaxContour = c

    return areaMaxContour, contour_area_max

def run(img):
    global lab_data
    size = (640, 480)
    camera_center_x = 320  # Center of the camera feed on the x-axis
    center_threshold_x = 150  # Allowable error in pixels to consider the ball centered on the x-axis
    target_color = ('yellow',)  # Assuming green is the target color
    power = 40
    power_sideways = 50
    
    img_copy = img.copy()
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    ball_detected = False
    
    for i in target_color:
        if i in lab_data:
            frame_mask = cv2.inRange(frame_lab,
                                     (lab_data[i]['min'][0], lab_data[i]['min'][1], lab_data[i]['min'][2]),
                                     (lab_data[i]['max'][0], lab_data[i]['max'][1], lab_data[i]['max'][2]))
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)
            
            if area_max > 1000:
                ball_detected = True
                (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)
                diameter = 2 * radius
                cv2.circle(img, (int(center_x), int(center_y)), int(radius), (0, 255, 0), 2)
                
                # If the diameter is over 200 pixels but less than 350, move forward without checking centering
                if 200 < diameter < 350:
                    motors[1].speed(power)
                    motors[2].speed(power)
                # Stop if the diameter is 350 pixels or more
                elif diameter >= 350:
                    motors.stop()
                # Adjust the car's direction based on the ball's position if the diameter is 200 pixels or less
                elif abs(center_x - camera_center_x) >= center_threshold_x:
                    if center_x < camera_center_x:  # Ball is to the left, rotate car to the left
                        motors[1].speed(-power_sideways)
                        motors[2].speed(power_sideways)
                    else:  # Ball is to the right, rotate car to the right
                        motors[1].speed(power_sideways)
                        motors[2].speed(-power_sideways)
                # If the ball is centered and diameter is less than or equal to 200 pixels, move forward
                else:  
                    motors[1].speed(power)
                    motors[2].speed(power)
                break
    
    if not ball_detected:
        motors.stop()
        
    return img

if __name__ == '__main__':
    load_config()
    camera.camera_open(correction=True)
    try:
        while True:
            img = camera.frame
            if img is not None:
                frame = img.copy()
                frame = run(frame)
                cv2.imshow('frame', frame)
                key = cv2.waitKey(1)
                if key == 27:  # ESC key
                    break
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        camera.camera_close()
        motors.stop()
        cv2.destroyAllWindows()
