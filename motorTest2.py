from robot_hat import Motors, Pin, PWM
#from robot_hat import MyMotor
import time

motors = Motors()

def main():
    while True:
        motors[2].speed(-35)
        time.sleep(0.001)

def destroy():
    motors.stop()
    print("Motors Stopped")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        destroy()

