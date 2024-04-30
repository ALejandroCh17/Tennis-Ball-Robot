#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/rally/robot-hat/TurboPi')
import cv2
import time
import math
import Camera
import numpy as np
import yaml_handle
from robot_hat import Motors, ADC, PWM, Ultrasonic, Pin, reset_mcu

us = Ultrasonic(Pin("D0"), Pin("D1"))
upperBound1 = 11.0
lowerBound1 = 7.0
upperBound2 = 11.0 #69.0
lowerBound2 = 7.0 #60.00
initialValue = us.read()

#Must be defined before motors, otherwise motors don't trigger
pwm_channel = 'P0'  # Example PWM channel
pwm = PWM(pwm_channel)
pwm.freq(800)  # Example frequency



# Initialize camera and motors
camera = Camera.Camera()
motors = Motors()
ir_sensor = ADC('A0') 
over_160 = False
diameter = 0
ball_detected = False


# Movement state
moving = False
move_time = 2
stop_time = 2
last_move_time = time.time()

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

def getAreaMaxContour(contours):
    # Implementation assumes this function returns the largest contour and its area
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

def check_ir_sensor():
    value = ir_sensor.read()
    voltage = value * 3.3 / 4095
    if voltage > 0.5:
        return True
    return False

def is_over_160():
    global diameter
    if diameter > 65: #65:
        return True
    else:
        return False
    
def trigger_pwm():
    print("PWM Triggered")
    pwm.pulse_width_percent(50)  # Set PWM pulse width percent, adjust as needed

def stop_pwm():
    pwm.pulse_width_percent(0)  # Stop PWM by setting pulse width to 0

def correct_blades():
    percent = 0.028 / 2
    while True:
        if not check_ir_sensor():
            pwm.pulse_width_percent(35)  # Activate motor in unsafe state
            time.sleep(percent)
            pwm.pulse_width_percent(0)
        else:
            pwm.pulse_width_percent(35)
            time.sleep(percent * 3)
            pwm.pulse_width_percent(0)  # Deactivate motor in safe state
            break 

        time.sleep(0.09)  # Small delay to reduce processing load

def trigger_blades():
    time.sleep(1)
    trigger_pwm()
    time.sleep(2)
    stop_pwm()
    correct_blades()

def rotate_robot():
    print("SCANNING")
    
    global moving, last_move_time
    power = 50
    if moving:
        if time.time() - last_move_time > move_time:
            motors.stop()
            moving = False
            last_move_time = time.time()
    else:
        if time.time() - last_move_time > stop_time:
            # Rotate the robot, change direction periodically
            motors[1].speed(50)
            motors[2].speed(-50)
            moving = True
            last_move_time = time.time()
    
    '''
    motors[1].speed(50)
    motors[2].speed(-50)
    start_time = time.perf_counter()
    interval = 2

    while time.perf_counter() - start_time < interval:
        pass

    motors.stop()
    '''


def detect_and_chase_ball(img):
    global moving
    global over_160
    global diameter
    global ball_detected
    size = (640, 480)
    camera_center_x = 320  # Center of the camera feed on the x-axis
    center_threshold_x = 150  # Allowable error in pixels to consider the ball centered on the x-axis
    target_color = ('yellow',)  # Assuming green is the target color
    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    power = 40
    power_sideways = 50

    img_copy = img.copy()
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    ball_detected = False

    for i in target_color:  
        frame_mask = cv2.inRange(frame_lab, (lab_data[i]['min'][0], lab_data[i]['min'][1], lab_data[i]['min'][2]), (lab_data[i]['max'][0], lab_data[i]['max'][1], lab_data[i]['max'][2]))
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        areaMaxContour, area_max = getAreaMaxContour(contours)
        
        if areaMaxContour is not None and area_max > 1000:  # try deleting areaMaxContour is not None 
            ball_detected = True
            #motors.stop()  # Stop the circular movement
            moving = True
            # Code to chase the ball goes here
            over_160 = False
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)
            diameter = 2 * radius
            cv2.circle(img, (int(center_x), int(center_y)), int(radius), (0, 255, 0), 2)
                
            # If the diameter is over 200 pixels but less than 350, move forward without checking centering
            if 200 < diameter < 350:
                motors[1].speed(power)
                motors[2].speed(power)
            # Stop if the diameter is 350 pixels or more
            elif diameter >= 350:
                print("Check 1")
                motors.stop()
            # Adjust the car's direction based on the ball's position if the diameter is 200 pixels or less
            elif abs(center_x - camera_center_x) >= center_threshold_x:
                if center_x < camera_center_x:  # Ball is to the left, rotate car to the left
                    print("Moving left", ball_detected, " ", diameter)
                    motors[1].speed(-power_sideways)
                    motors[2].speed(power_sideways)
                else:  # Ball is to the right, rotate car to the right
                    print("Moving right")
                    motors[1].speed(power_sideways)
                    motors[2].speed(-power_sideways)
            # If the ball is centered and diameter is less than or equal to 200 pixels, move forward
            else:  
                motors[1].speed(power)
                motors[2].speed(power)

            over_160 = is_over_160()
        break    

    if not ball_detected: #try adding an additional boolean that detects when the car is in the process of chasing a ball
        if over_160:
            print("over 160")
            if check_ir_sensor():
                print("Check 1 ", diameter)
                motors.stop()
                trigger_blades()
                time.sleep(1)
                    
        else:
            print("STUCK ", diameter)
            motors.stop()
            time.sleep(1)

    return img

if __name__ == '__main__':
    load_config()
    camera.camera_open(correction=True)
    correct_blades()
    #initial_it = True
    #moving = False
    #last_move_time = time.time()
    try:
        while True:
            img = camera.frame
            if img is not None:
                frame = img.copy()
                frame = detect_and_chase_ball(frame)
                if not ball_detected: #or initial_it:
                    rotate_robot()
                #initial_it = False
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
