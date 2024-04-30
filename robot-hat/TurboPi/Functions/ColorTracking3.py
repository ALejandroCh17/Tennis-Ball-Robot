#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/rally/robot-hat/TurboPi')
import cv2
import time
import math
import signal
import Camera
import argparse
import threading
import numpy as np
import yaml_handle
#import HiwonderSDK.PID as PID
#import HiwonderSDK.Misc as Misc
#import HiwonderSDK.Board as Board
#import HiwonderSDK.mecanum as mecanum

# 颜色追踪
size = (640, 480)
target_color = ()
lab_data = None

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
    global color_radius
    global color_center_x, color_center_y
    
    img_copy = img.copy()
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    
    area_max = 0
    areaMaxContour = 0

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
        (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)
        color_center_x = int(center_x)
        color_center_y = int(center_y)
        color_radius = radius

        cv2.circle(img, (color_center_x, color_center_y), int(radius), (0, 255, 0), 2)

        diameter = 2 * radius  # Calculate the diameter
        print("Diameter of detected object: {:.2f} pixels".format(diameter))
    else:
        color_radius = 0
        color_center_x = -1
        color_center_y = -1

    return img

if __name__ == '__main__':
    load_config()
    target_color = ('yellow',)
    camera = Camera.Camera()
    camera.camera_open(correction=True)
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
    camera.camera_close()
    cv2.destroyAllWindows()


