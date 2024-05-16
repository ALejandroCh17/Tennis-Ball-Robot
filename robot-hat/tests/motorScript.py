from robot_hat import PWM
from robot_hat import reset_mcu
import time

# Define the PWM channel for the single pin
channel = 'P0'

# Initialize the PWM object
pwm = PWM(channel)

# Set the frequency and pulse width for the PWM channel
pwm.freq(800)  # Adjust this value as needed
pwm.pulse_width_percent(100)

# Counter for the ticks
tick_counter = 0

# Function to set the state of the PWM channel
def set_state():
    global tick_counter
    tick_counter += 1
    if tick_counter % 5 == 0:
        pwm.pulse_width_percent(0)  # Turn off the motor every 5th tick
    else:
        pwm.pulse_width_percent(50)  # Otherwise, keep the motor on

# Main loop
while True:
    set_state()
    time.sleep(1/800)  # Sleep for the duration of one tick
    pwm.pulse_width_percent(0)
reset_mcu()
