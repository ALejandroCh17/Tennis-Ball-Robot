import cv2
import tensorflow as tf
import numpy as np
import time

# Load the model
model_path = r'C:\Users\16199\Downloads\my_model_before_increase.h5'
model = tf.keras.models.load_model(model_path)

# Initialize camera and motors
sys.path.append('/home/robot2/robot-hat/TurboPi')
import Camera
from robot_hat import Motors

camera = Camera.Camera()
motors = Motors()

# Function to make the robot spin
def spin_motors(speed):
    motors[1].speed(speed)  # Right motor
    motors[2].speed(-speed)  # Left motor spins in opposite direction for spinning in place

# Start capturing video
camera.camera_open()

try:
    while True:
        frame = camera.frame
        if frame is not None:
            # Preprocess the frame according to your model's requirements
            frame_resized = cv2.resize(frame, (128, 128))  # Resize to match model input
            frame_normalized = frame_resized / 255.0  # Normalize pixel values
            frame_batch = np.expand_dims(frame_normalized, axis=0)  # Add batch dimension
            
            # Predict
            prediction = model.predict(frame_batch)
            
            # Check prediction confidence
            if prediction[0][0] >= 0.51:
                print("High confidence detected. Stopping video feed.")
                motors.stop()  # Stop the motors
                break  # Exit the loop if confidence is high
            
            # Process the prediction here (e.g., display it)
            print("Prediction: ", prediction)
            
            # Spin the robot
            spin_motors(40)  # Adjust speed as needed for your robot

            # Show the frame (optional)
            cv2.imshow('Video', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit on pressing 'q'
                break

        else:
            time.sleep(0.01)  # Wait a bit if the frame is not captured
except Exception as e:
    print(e)
finally:
    camera.camera_close()
    motors.stop()
    cv2.destroyAllWindows()
