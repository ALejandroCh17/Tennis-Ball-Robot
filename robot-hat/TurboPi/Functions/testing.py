#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/rally/robot-hat/TurboPi/')
import cv2
import time
import math
import Camera
import numpy as np
import yaml_handle
from robot_hat import Motors, ADC, PWM, Ultrasonic, Pin, reset_mcu
from picamera2 import Picamera2, Preview

#us = Ultrasonic(Pin("D0"), Pin("D1"))
upperBound1 = 11.0
lowerBound1 = 7.0
upperBound2 = 11.0 #69.0
lowerBound2 = 7.0 #60.00
#initialValue = us.read()

#Must be defined before motors, otherwise motors don't trigger
pwm_channel = 'P0'  # Example PWM channel
pwm = PWM(pwm_channel)
pwm.freq(800)  # Example frequency



# Initialize camera and motors
camera = Picamera2(0)  # Use index 0 for the available camera device
camera.configure(camera.create_preview_configuration())
motors = Motors()
ir_sensor = ADC('A0')
right_sensor = ADC('A1')
left_sensor = ADC('A2') 
over_160 = False
diameter = 0
last_seen_time = 0
blade_triggered = True

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

def check_ir_sensor():
    value = ir_sensor.read()
    voltage = value * 3.3 / 4095
    if voltage > 0.5:
        return True
    return False

def check_right_sensor():
    value = right_sensor.read()
    voltage = value * 3.3 / 4095
    if voltage > 0.5:
        return False
    return True

def check_left_sensor():
    value = left_sensor.read()
    voltage = value * 3.3 / 4095
    if voltage > 0.5:
        return False
    return True

def obstacle_right():
    power_sideways = 50
    power = 40
    motors[1].speed(-power_sideways)
    motors[2].speed(-power_sideways)
    time.sleep(1.5)
    motors[1].speed(-power_sideways)
    motors[2].speed(power_sideways)
    time.sleep(1.5)

    motors.stop()
    global blade_triggered
    blade_triggered = True


def obstacle_left():
    power_sideways = 50
    power = 40
    motors[1].speed(-power_sideways)
    motors[2].speed(-power_sideways)
    time.sleep(1.5)
    motors[1].speed(power_sideways)
    motors[2].speed(-power_sideways)
    time.sleep(1.5)

    motors.stop()
    global blade_triggered
    blade_triggered = True


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
    print("Correcting blades")
    percent = 0.028 / 2
    while True:
        if not check_ir_sensor():
            pwm.pulse_width_percent(35)  # Activate motor in unsafe state
            time.sleep(percent)
            pwm.pulse_width_percent(0)
        else:
            pwm.pulse_width_percent(35)
            time.sleep(percent * 4)
            pwm.pulse_width_percent(0)  # Deactivate motor in safe state
            break 

        time.sleep(0.09)  # Small delay to reduce processing load

def trigger_blades():
    time.sleep(0.5)
    trigger_pwm()
    time.sleep(1.5)
    pwm.pulse_width_percent(0)
    time.sleep(1)
    correct_blades()
    global blade_triggered
    blade_triggered = True

def patrol():
    global blade_triggered
    if ball_detected:
        blade_triggered = False
        return
    power_sideways = 50
    motors[1].speed(power_sideways)
    motors[2].speed(-power_sideways)
    time.sleep(1.0)
    motors[1].speed(0)
    motors[2].speed(0)
    time.sleep(1.0)
    
    

def fallback_strategy():
    power_sideways = 50
    motors[1].speed(-power_sideways)
    motors[2].speed(power_sideways)
    time.sleep(2)
    motors.stop()

def run(img):
    global lab_data
    global over_160
    global diameter
    global ball_detected
    global last_seen_time
    global good_to_patrol
    global blade_triggered

    size = (640, 480)
    camera_center_x = 320  # Center of the camera feed on the x-axis
    center_threshold_x = 150  # Allowable error in pixels to consider the ball centered on the x-axis
    target_color = ('yellow',)  # Assuming green is the target color
    power = 40
    power_sideways = 50
    timeout = 1.0 
    ball_detected = False
    
    img_copy = img.copy()
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)


    
    
    for i in target_color:
        if i in lab_data:
            frame_mask = cv2.inRange(frame_lab,
                                     (lab_data[i]['min'][0], lab_data[i]['min'][1], lab_data[i]['min'][2]),
                                     (lab_data[i]['max'][0], lab_data[i]['max'][1], lab_data[i]['max'][2]))
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)
            
            #print("BALL ", ball_detected)
            

            if area_max > 1000:
                ball_detected = True
                print(ball_detected)
                blade_triggered = False
                over_160 = False
                last_seen_time = time.time()
                (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)
                diameter = 2 * radius
                cv2.circle(img, (int(center_x), int(center_y)), int(radius), (0, 255, 0), 2)
                
                # If the diameter is over 200 pixels but less than 350, move forward without checking centering
                if 200 < diameter < 350:
                    motors[1].speed(power)
                    motors[2].speed(power)
                # Stop if the diameter is 350 pixels or more
                elif diameter >= 350:
                    #print("Check 1")
                    motors.stop()
                # Adjust the car's direction based on the ball's position if the diameter is 200 pixels or less
                elif abs(center_x - camera_center_x) >= center_threshold_x:
                    if center_x < camera_center_x:  # Ball is to the left, rotate car to the left
                        #print("Moving left", ball_detected, " ", diameter)
                        motors[1].speed(-power_sideways)
                        motors[2].speed(power_sideways)

                        if check_right_sensor():
                            obstacle_right()

                        if check_left_sensor():
                            obstacle_left()
                    else:  # Ball is to the right, rotate car to the right
                        #print("Moving right")
                        motors[1].speed(power_sideways)
                        motors[2].speed(-power_sideways)

                        if check_right_sensor():
                            obstacle_right()

                        if check_left_sensor():
                            obstacle_left()    

                # If the ball is centered and diameter is less than or equal to 200 pixels, move forward
                else:
                    #print("moving forward")
                    time.sleep(0.01)
                    motors.stop()
                    motors[1].speed(power)
                    motors[2].speed(power)
                    #print("After moving forward")

                    if check_ir_sensor():
                        #print("Triggering scooper ", diameter)
                        motors.stop()
                        trigger_blades()
                        time.sleep(1)

                    if check_right_sensor():
                        obstacle_right()

                    if check_left_sensor():
                            obstacle_left()

                over_160 = is_over_160()
                break
        
        if check_ir_sensor():
            #print("Triggering scooper ", diameter)
            motors.stop()
            trigger_blades()
            time.sleep(1)
        elif time.time() - last_seen_time > timeout:
            motors.stop()
            #fallback_strategy()
            #print("Timeout. Stopping")
        
        
        if check_right_sensor():
            obstacle_right()

        if check_left_sensor():
            obstacle_left()

        if not ball_detected and blade_triggered:
            #print("Patrolling")
                patrol()



        break


        '''
        if not ball_detected:
            if check_ir_sensor():
                print("Check 1 ", diameter)
                motors.stop()
                trigger_blades()
                time.sleep(1)
            elif time.time() - last_seen_time > timeout:
                motors.stop()
                #fallback_strategy()
                #print("Timeout. Stopping")
            break
        '''


    '''   
    if not ball_detected:
        if over_160:
            if check_ir_sensor():
                print("Check 1 ", diameter)
                motors.stop()
                trigger_blades()
                time.sleep(1)
                    
        else:
            print("Check 2 ", diameter)
            motors.stop()   
            time.sleep(1)
    '''
    
    '''
    if not ball_detected and diameter < 160:
        motors.stop()
    elif not ball_detected and diameter > 160 and check_ir_sensor():
        motors.stop()
    '''

    '''
    if not ball_detected:
        motors.stop()
    '''
    return img

if __name__ == '__main__':
    load_config()
    correct_blades()
    camera.start()
    #camera.camera_open(correction=True)
    camera.cap = cv2.VideoCapture(-1)
    camera.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    camera.cap.set(cv2.CAP_PROP_FPS, 30)
    camera.cap.set(cv2.CAP_PROP_SATURATION, 40)
    #camera.correction = correction
    #self.opened = True
    try:
        while True:
                # Capture an image using picamera2
            camera.capture_file('/tmp/image.jpg')
            
            # Read the captured image using OpenCV
            frame = cv2.imread('/tmp/image.jpg')
            if frame is not None:
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
        camera.stop()
        motors.stop()
        cv2.destroyAllWindows()
