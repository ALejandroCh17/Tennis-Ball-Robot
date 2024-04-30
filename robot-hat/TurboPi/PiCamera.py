import picamera
import picamera.array
import numpy as np
import threading

class PiCamera:
    def __init__(self, resolution=(640, 480)):
        self.camera = picamera.PiCamera()
        self.camera.resolution = resolution
        self.frame = None
        self.opened = False
        self.stream = picamera.array.PiRGBArray(self.camera)
        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        
    def camera_open(self):
        self.opened = True
        self.th.start()
        
    def camera_close(self):
        self.opened = False
        self.th.join()
        self.camera.close()

    def camera_task(self):
        while self.opened:
            self.stream.seek(0)
            self.stream.truncate()
            self.camera.capture(self.stream, format='bgr', use_video_port=True)
            self.frame = self.stream.array

