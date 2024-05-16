from robot_hat import PWM, reset_mcu
import time

try:
    # Define the PWM channel for the single pin
    channel = 'P0'

    # Initialize the PWM object
    pwm = PWM(channel)

    # Set the frequency and pulse width for the PWM channel
    pwm.freq(800)  # Adjust this value as needed
    pwm.pulse_width_percent(50)  # Set the pulse width to 50%

    while True:
        time.sleep(1)  # Keep the program running

except KeyboardInterrupt:
    # This will run when a keyboard interrupt (CTRL+C) is detected
    print("Stopping the motor")
    pwm.pulse_width_percent(0)  # Stop the motor
    #reset_mcu()  # Reset MCU if necessary

