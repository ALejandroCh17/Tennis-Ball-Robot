def run(img):
    global lab_data
    size = (640, 480)
    camera_center_x = 320  # Center of the camera feed on the x-axis
    center_threshold_x = 150  # Allowable error in pixels to consider the ball centered on the x-axis
    target_color = ('green',)  # Assuming green is the target color
    
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
                (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)
                diameter = 2 * radius
                cv2.circle(img, (int(center_x), int(center_y)), int(radius), (0, 255, 0), 2)
                
                # If the diameter is over 200 pixels but less than 350, move forward without checking centering
                if 200 < diameter < 350:
                    motors[1].speed(30)
                    motors[2].speed(30)
                # Stop if the diameter is 350 pixels or more
                elif diameter >= 350:
                    motors.stop()
                # Adjust the car's direction based on the ball's position if the diameter is 200 pixels or less
                elif abs(center_x - camera_center_x) >= center_threshold_x:
                    if center_x < camera_center_x:  # Ball is to the left, rotate car to the left
                        motors[1].speed(-30)
                        motors[2].speed(30)
                    else:  # Ball is to the right, rotate car to the right
                        motors[1].speed(30)
                        motors[2].speed(-30)
                # If the ball is centered and diameter is less than or equal to 200 pixels, move forward
                else:  
                    motors[1].speed(30)
                    motors[2].speed(30)
                break
    
    if not ball_detected:
        motors.stop()
        
    return img

