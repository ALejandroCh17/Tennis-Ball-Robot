from ultralytics import YOLO

model = YOLO('yolo8x')

model.predict('input_videos/image.png', save=True)