from robot_hat import PWM
import time

# Define the PWM channels for the four input pins of the ULN2003 module
channels = ['P0', 'P1', 'P2', 'P3']

# Initialize the PWM objects
pwms = [PWM(ch) for ch in channels]

# Define the sequence for half-step driving
halfstep_seq = [
    [1,0,0,0], [1,1,0,0], [0,1,0,0], [0,1,1,0], [0,0,1,0], [0,0,1,1], [0,0,0,1], [1,0,0,1]
]

# Increase the speed by reducing the delay
speed = 0.005 # Smaller values mean faster speed

# Loop through the sequence to rotate the motor
for i in range(512): # Adjust number for desired steps
    for halfstep in range(8):
        for pin in range(4):
            pwms[pin].pulse_width_percent(halfstep_seq[halfstep][pin]*100)
            time.sleep(speed)

# Reset the PWM channels
for pwm in pwms:
    pwm.pulse_width_percent(0)

