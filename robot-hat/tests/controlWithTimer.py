import curses
from robot_hat import Motors, PWM, Pin
import time
import subprocess

# Initialize PWM control for another device, if needed
pwm_channel = 'P0'  # Example PWM channel
pwm = PWM(pwm_channel)
pwm.freq(800)  # Example frequency

motors = Motors()
power = 50

def forward():
    motors[1].speed(power)  # Motor 1 forward
    motors[2].speed(power)  # Motor 2 backward

def backward():
    motors[1].speed(-power)  # Motor 1 forward
    motors[2].speed(-power)  # Motor 2 backward

def clockwise():
    motors[1].speed(power)  # Motor 1 forward
    motors[2].speed(-power)  # Motor 2 backward

def counter_clockwise():
    motors[1].speed(-power)  # Motor 1 forward
    motors[2].speed(power)  # Motor 2 backward

def trigger_pwm():
    print("PWM Triggered")
    pwm.pulse_width_percent(50)  # Set PWM pulse width percent, adjust as needed

def stop_pwm():
    pwm.pulse_width_percent(0)  # Stop PWM by setting pulse width to 0

def destroy():
    motors.stop()
    stop_pwm()
    print("Motors and PWM stopped.")

# Mapping from keys to functions
actions = {
    curses.KEY_UP: forward,
    curses.KEY_DOWN: backward,
    curses.KEY_LEFT: counter_clockwise,
    curses.KEY_RIGHT: clockwise,
    ord('s'): trigger_pwm,  # Add the trigger_pwm action for "s" key
    ord('S'): trigger_pwm,  # Optionally, if you want to ensure both lowercase and uppercase "S" trigger the action
}

print("Remote Control Program")

def main(window):
    next_key = None
    start_time = None

    while True:
        curses.halfdelay(1)
        if next_key is None:
            key = window.getch()
        else:
            key = next_key
            next_key = None

        if key != -1:
            # KEY PRESSED
            start_time = time.time()  # Start timer
            curses.halfdelay(3)
            action = actions.get(key)
            if action is not None:
                action()
            next_key = key
            while next_key == key:
                next_key = window.getch()
            # KEY RELEASED
            end_time = time.time()  # End timer
            duration = end_time - start_time  # Calculate duration
            print(f"Key held for {duration:.2f} seconds.")
            motors.stop()
            stop_pwm()  # Optional: stop PWM when any key is released

curses.wrapper(main)

