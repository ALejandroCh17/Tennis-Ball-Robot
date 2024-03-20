from robot_hat import Motors, Pin, PWM
#from robot_hat import MyMotor
import time

motors = Motors()

pin = PWM("P5")

def main():
    while True:
        motors[1].speed(30)
        time.sleep(0.001)
        pin.freq(1000)
        pin.pulse_width_percent(100)

def destroy():
    motors.stop()
    print("Motors Stopped")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        destroy()

