import curses
from robot_hat import Motors, Pin
import time
import subprocess
#from robot_hat import reset_mcu
#reset_mcu()


motors = Motors()
power = 35

def forward():
    motors[1].speed(power)  # Motor 1 forward
    motors[2].speed(power) # Motor 2 backward
    #time.sleep(0.01)

def backward():
    motors[1].speed(-power)  # Motor 1 forward
    motors[2].speed(-power) # Motor 2 backward
    #time.sleep(0.01)

def clockwise():
    motors[1].speed(power)  # Motor 1 forward
    motors[2].speed(-power) # Motor 2 backward
    #time.sleep(0.01)

def counter_clockwise():
    motors[1].speed(-power)  # Motor 1 forward
    motors[2].speed(power) # Motor 2 backward
    #time.sleep(0.01)

def destroy():
    # Stop motors when Ctrl+C is pressed
    motors.stop()
    print("Motors stopped.")

actions = {
    curses.KEY_UP:    forward,
    curses.KEY_DOWN:  backward,
    curses.KEY_LEFT:  counter_clockwise,
    curses.KEY_RIGHT: clockwise,
}


print("Remote Control Program")

def main(window):
    next_key = None
    while True:
        curses.halfdelay(1)
        if next_key is None:
            key = window.getch()
        else:
            key = next_key
            next_key = None
        if key != -1:
            # KEY PRESSED
            curses.halfdelay(3)
            action = actions.get(key)
            if action is not None:
                action()
            next_key = key
            while next_key == key:
                next_key = window.getch()
            # KEY RELEASED
            motors.stop()

curses.wrapper(main)
