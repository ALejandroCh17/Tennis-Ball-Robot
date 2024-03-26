import cv2
import numpy as np

# Constants for distance estimation (these need to be calibrated for your setup)
BALL_DIAMETER_CM = 6.7  # Diameter of a tennis ball in centimeters
FOCAL_LENGTH_PIXELS = 800  # This needs to be calibrated

# Estimate the distance to the ball using the apparent radius in pixels
def estimate_distance(radius_pixels):
    if radius_pixels > 0:  # Prevent division by zero
        return (FOCAL_LENGTH_PIXELS * BALL_DIAMETER_CM) / (radius_pixels * 2)
    else:
        return -1  # Return -1 if the ball is not detected or radius is zero

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
        
        # Estimate and print the distance
        distance = estimate_distance(radius)
        if distance > 0:
            print(f"Estimated distance to the ball: {distance:.2f} cm")
        else:
            print("Ball not detected")

    # Display the resulting frame
    cv2.imshow('Tennis Ball Tracker', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
