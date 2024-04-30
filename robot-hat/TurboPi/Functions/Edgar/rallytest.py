from picamera2 import Picamera2, Preview

import cv2

import numpy as np

from time import sleep



# Initialize the PiCamera2 object

picam0 = Picamera2(0)  # Use index 0 for the available camera device

#picam0.start_preview(Preview.QTGL)

picam0.start()



# Load the tennis ball cascade classifier

tennis_cascade = cv2.CascadeClassifier('cascade_12stages_24dim_0_25far.xml')



try:

    while True:

        # Capture an image using picamera2

        picam0.capture_file('/tmp/image.jpg')

        

        # Read the captured image using OpenCV

        frame = cv2.imread('/tmp/image.jpg')

        

        # Convert the image to grayscale

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        

        # Perform tennis ball detection using the cascade classifier

        tennis_balls = tennis_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        

        # Draw bounding boxes around detected tennis balls

        for (x, y, w, h) in tennis_balls:

        

            center_x = x + w // 2

            center_y = y + h // 2

            radius = min(w, h) // 2

            cv2.circle(frame, (center_x, center_y), radius, (0, 255, 0), 2)

            print(radius * 2)

        

        # Display the processed image

        cv2.imshow('Tennis Ball Detection', frame)

        

        # Wait for a key press and check if it's the 'q' key to exit

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):

            break



except KeyboardInterrupt:

    pass



finally:

    picam0.stop()

    picam0.stop_preview()

    cv2.destroyAllWindows()

