import cv2
import subprocess
import tempfile
import os

class Camera:
    def __init__(self, resolution=(640, 480)):
        self.resolution = resolution
        self.frame = None

    def capture_frame(self):
        # Use libcamera-still to capture an image to a temporary file
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmpfile:
            capture_command = ['libcamera-still', '-o', tmpfile.name, '--nopreview', '--width', str(self.resolution[0]), '--height', str(self.resolution[1]), '-t', '1', '-n']
            subprocess.run(capture_command, check=True)
            # Load the image with OpenCV
            self.frame = cv2.imread(tmpfile.name)
            os.remove(tmpfile.name)  # Clean up the temporary file

    def camera_open(self):
        # This method is kept for compatibility but does nothing in this context.
        pass

    def camera_close(self):
        # This method is kept for compatibility but does nothing in this context.
        pass

