import cv2
import tensorflow as tf
import numpy as np
import time  # Import the time module

# Load the model
model = tf.keras.models.load_model(r'C:\Users\16199\Downloads\my_model_before_increase.h5')

# Initialize webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Preprocess the frame according to your model's requirements
    # Example for resizing and normalization
    frame_resized = cv2.resize(frame, (128, 128))  # Resize to match model input
    frame_normalized = frame_resized / 255.0  # Normalize if required by your model
    frame_batch = np.expand_dims(frame_normalized, axis=0)  # Add batch dimension
    
    # Predict
    prediction = model.predict(frame_batch)

    # Check prediction confidence
    if prediction[0][0] >= 0.51:
        print("High confidence detected. Stopping video feed.")
        break  # Exit the loop if confidence is high
    
    # Process the prediction here (e.g., display it)
    print(prediction)
    
    # Show the frame (optional)
    cv2.imshow('Video', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit on pressing 'q'
        break

    #time.sleep(1) 

cap.release()
cv2.destroyAllWindows()
