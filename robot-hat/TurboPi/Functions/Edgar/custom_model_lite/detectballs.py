import numpy as np
import cv2
import tensorflow as tf

# Load the TensorFlow Lite model.
interpreter = tf.lite.Interpreter(model_path="detect.tflite")
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Load the label map if available.
# Replace labels_path with your label map file path.
labels = []
with open("labelmap.txt", "r") as f:
    labels = [line.strip() for line in f.readlines()]

# Define the threshold for detection confidence.
threshold = .50
# Function to preprocess input image for the model.
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

# Main function to capture video from webcam and perform object detection.
def main():
    cap = cv2.VideoCapture(0)  # Use the default camera (usually webcam)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Perform object detection.
        detections = detect_objects(frame)

        # Draw bounding boxes around detected objects.
        for label, confidence in detections:
            cv2.putText(frame, '{}: {:.2f}'.format(label, confidence), (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            print("tennis ball in view")
        
        cv2.imshow('Object Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
