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
import tensorflow as tf

# Load the TensorFlow Lite model.
interpreter = tf.lite.Interpreter(model_path="/home/rally/robot-hat/TurboPi/Functions/Edgar/custom_model_lite/detect.tflite")
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Load the label map if available.
# Replace labels_path with your label map file path.
labels = []
with open("/home/rally/robot-hat/TurboPi/Functions/Edgar/custom_model_lite/labelmap.txt", "r") as f:
    labels = [line.strip() for line in f.readlines()]

# Define the threshold for detection confidence.
threshold = .50
balls_detected = False

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
over_160 = False
diameter = 0
last_seen_time = 0

def object_detected(frame):
    detections = detect_objects(frame)
    # Draw bounding boxes around detected objects.
    if(detections != None):
        for label, confidence in detections:
            cv2.putText(frame, '{}: {:.2f}'.format(label, confidence), (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            print("tennis ball in view")
            global balls_detected 
            balls_detected = True

    return balls_detected
            

def preprocess_image(image):
    input_shape = input_details[0]['shape']
    img = cv2.resize(image, (input_shape[2], input_shape[1]))  # Swap dimensions for TensorFlow Lite model
    img = img / 255.0
    img = np.expand_dims(img, axis=0)
    return img.astype(input_details[0]['dtype'])

# Function to perform object detection on input image.
def detect_objects(image):
    input_data = preprocess_image(image)
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    output_data = interpreter.get_tensor(output_details[0]['index'])
    output_data = np.squeeze(output_data)

    # Filter out detections with confidence scores lower than threshold.
    detections = []
    for i, score in enumerate(output_data):
        if score > threshold:
            class_id = int(i)
            confidence = score
            label = labels[class_id] if class_id < len(labels) else 'Class {}'.format(class_id)
            detections.append((label, confidence))
    return detections

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

    size = (640, 480)
    camera_center_x = 320  # Center of the camera feed on the x-axis
    center_threshold_x = 150  # Allowable error in pixels to consider the ball centered on the x-axis
    target_color = ('yellow',)  # Assuming green is the target color
    power = 60
    power_sideways = 50
    timeout = 1.0

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

        if not ball_detected:
            if check_ir_sensor():
                print("Check 1 ", diameter)
                motors.stop()
                trigger_blades()
                global balls_detected 
                balls_detected = True #this is the global variable for deep learning
                time.sleep(1)
            elif time.time() - last_seen_time > timeout:
                motors.stop()
                #fallback_strategy()
                #print("Timeout. Stopping")
            break

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
            
            object_detected(frame)
            if frame is not None and balls_detected:
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
