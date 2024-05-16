from robot_hat import Motors, Pin
import time

# Create motor object
motors = Motors()

# Initialize line tracking sensor

def main():
    while True:
        # print("value", line_track.value())
        # time.sleep(0.01)
        # If line is detected
        motors[1].speed(30)  # Motor 1 forward
        motors[2].speed(-30) # Motor 2 backward
        time.sleep(0.01)

def destroy():
    # Stop motors when Ctrl+C is pressed
    motors.stop()
    print("Motors stopped.")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        destroy()
