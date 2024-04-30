import sys
sys.path.append('/home/rally/robot-hat/TurboPi')
import cv2
import time
import numpy as np
import Camera
from robot_hat import Motors, ADC, PWM, Ultrasonic, Pin, reset_mcu
import yaml_handle

# Initialization and configuration
def initialize_system():
    global us, pwm, camera, motors, ir_sensor, lab_data
    us = Ultrasonic(Pin("D0"), Pin("D1"))
    pwm = PWM('P0')
    pwm.freq(800)
    camera = Camera.Camera()
    motors = Motors()
    ir_sensor = ADC('A0')
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

def cleanup():
    camera.camera_close()
    motors.stop()
    cv2.destroyAllWindows()

# Utility functions
def read_ultrasonic():
    return us.read()

def process_image(img):
    size = (640, 480)
    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    return frame_lab

def find_ball(frame_lab):
    for color in ('yellow',):  # Assuming yellow is the target color
        if color in lab_data:
            frame_mask = cv2.inRange(frame_lab,
                                     (lab_data[color]['min'][0], lab_data[color]['min'][1], lab_data[color]['min'][2]),
                                     (lab_data[color]['max'][0], lab_data[color]['max'][1], lab_data[color]['max'][2]))
            return process_contours(frame_mask)
    return None, 0

def process_contours(mask):
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    return get_area_max_contour(contours)

def get_area_max_contour(contours):
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area = abs(cv2.contourArea(c))
        if contour_area > contour_area_max and contour_area > 300:
            contour_area_max = contour_area
            area_max_contour = c
    return area_max_contour, contour_area_max

def motor_decision_logic(areaMaxContour, area_max):
    global over_160, diameter
    power = 40
    power_sideways = 50
    camera_center_x = 320  # Center of the camera feed on the x-axis
    center_threshold_x = 150  # Allowable error in pixels to consider the ball centered on the x-axis

    (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)
    diameter = 2 * radius
    cv2.circle(img, (int(center_x), int(center_y)), int(radius), (0, 255, 0), 2)

    # Determine movement based on the diameter of the ball
    if 200 < diameter < 350:
        # If the ball is somewhat close but not too close, move forward
        motors[1].speed(power)
        motors[2].speed(power)
    elif diameter >= 350:
        # If the ball is very close, stop the motors
        print("Ball is very close. Stopping.")
        motors.stop()
    else:
        # Check if the ball is sufficiently centered to move forward
        if abs(center_x - camera_center_x) >= center_threshold_x:
            if center_x < camera_center_x:  # Ball is to the left
                print("Ball to the left. Adjusting left.")
                motors[1].speed(-power_sideways)
                motors[2].speed(power_sideways)
            else:  # Ball is to the right
                print("Ball to the right. Adjusting right.")
                motors[1].speed(power_sideways)
                motors[2].speed(-power_sideways)
        else:
            # Ball is centered, move forward
            print("Ball centered. Moving forward.")
            motors[1].speed(power)
            motors[2].speed(power)

    # Update global state based on the diameter of the ball
    over_160 = diameter > 160
    if over_160 and check_ir_sensor():
        print("IR sensor triggered with large object detected. Handling object.")
        handle_large_object()  # Function to handle situation when both conditions are met


# Main function
def run(img):
    frame_lab = process_image(img)
    area_max_contour, area_max = find_ball(frame_lab)
    motor_decision_logic(area_max_contour, area_max)
    return img

# Main Execution
if __name__ == '__main__':
    initialize_system()
    camera.camera_open(correction=True)
    try:
        while True:
            img = camera.frame
            if img:
                frame = run(img.copy())
                cv2.imshow('frame', frame)
                if cv2.waitKey(1) == 27:  # ESC to quit
                    break
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
