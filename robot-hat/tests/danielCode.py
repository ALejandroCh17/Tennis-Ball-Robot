import cv2
import numpy as np
from robot_hat import Motors


# Initialize the Motors
motors = Motors()
motors.set_left_id(1)  # Assuming this is necessary for your specific robot configuration
motors.set_right_id(2)

# Constants for distance estimation (these need to be calibrated for your setup)
BALL_DIAMETER_CM = 6.7  # Diameter of a tennis ball in centimeters
FOCAL_LENGTH_PIXELS = 800  # This needs to be calibrated

# Estimate the distance to the ball using the apparent radius in pixels
def estimate_distance(radius_pixels):
    if radius_pixels > 0:  # Prevent division by zero
        return (FOCAL_LENGTH_PIXELS * BALL_DIAMETER_CM) / (radius_pixels * 2)
    else:
        return -1  # Return -1 if the ball is not detected or radius is zero

# Compute the motor speeds based on the ball's position
def compute_motor_speeds(center_x, frame_width, distance):
    MAX_SPEED = 60
    DISTANCE_THRESHOLD = 50
    distance_factor = max(0, min(1, (distance - DISTANCE_THRESHOLD) / DISTANCE_THRESHOLD))

    # Calculate speed adjustments based on the ball's position
    if center_x < frame_width // 3:
        left_speed = distance_factor * MAX_SPEED
        right_speed = 0  # Stop the right motor to turn
    elif center_x > (frame_width // 3) * 2:
        right_speed = distance_factor * MAX_SPEED
        left_speed = 0  # Stop the left motor to turn
    else:
        left_speed = right_speed = distance_factor * MAX_SPEED

    # Set motor speeds
    motors.speed(left_speed, right_speed)
# Overlay motor speeds and distance on the frame
def overlay_info(frame, distance, left_speed, right_speed):
    cv2.putText(frame, f"Distance to ball: {distance:.2f} cm", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"Left Motor Speed: {left_speed:.2f}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"Right Motor Speed: {right_speed:.2f}", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

# Initialize the webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_lower = np.array([29, 86, 6], np.uint8)
    green_upper = np.array([64, 255, 255], np.uint8)
    
    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, green_lower, green_upper)
    
    # Find contours and the maximum contour
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_contour = max(contours, key=cv2.contourArea) if contours else None
    
    if max_contour is not None and cv2.contourArea(max_contour) > 500:
        (x, y), radius = cv2.minEnclosingCircle(max_contour)
        center = (int(x), int(y))
        radius = int(radius)

        # Draw the circle and centroid on the frame
        cv2.circle(frame, center, radius, (0, 255, 0), 2)
        
        # Estimate the distance
        distance = estimate_distance(radius)
        # Compute the motor speeds
        motor_speeds = compute_motor_speeds(center[0], frame.shape[1], distance)
        # Overlay the information on the frame
        #overlay_info(frame, distance, motors[0], motors[1])
        
    else:
        # If the ball is not found, display this information
        cv2.putText(frame, "Ball not detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # Display the resulting frame with overlays
    cv2.imshow('Tennis Ball Tracker', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()

