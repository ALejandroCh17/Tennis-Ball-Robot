from robot_hat import PWM
from robot_hat import reset_mcu
import time

# Define the PWM channels for the four input pins of the ULN2003 module
channels = ['P0', 'P1', 'P2', 'P3']

# Initialize the PWM objects
pwms = [PWM(ch) for ch in channels]

# Set the frequency and pulse width for each PWM channel
for pwm in pwms:
    pwm.freq(800)  # Adjust this value as needed
    pwm.pulse_width_percent(100)

# Define the sequence for the 28BYJ-48 stepper motor in full-step mode
rev_sequence = [[1,1,0,0], [0,1,1,0], [0,0,1,1], [1,0,0,1]]
sequence = [[1,0,0,1], [0,0,1,1], [0,1,1,0], [1,1,0,0]]

# Function to set the state of the PWM channels
def set_state(state):
    for i in range(4):
        if state[i] == 1:
            pwms[i].pulse_width_percent(50)
        else:
            pwms[i].pulse_width_percent(0)

# Main loop
step = 0
while True:
    set_state(sequence[step])
    time.sleep(0.0005)  # Adjust this value as needed
    step = (step + 1) % 4  # Change this to 4 because we're now using full-step mode

reset_mcu()

