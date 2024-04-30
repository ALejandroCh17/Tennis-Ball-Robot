from robot_hat import PWM, ADC, Pin, reset_mcu
import time

# Motor control setup
channel = 'P0'
pwm = PWM(channel)
pwm.freq(800)  # Set the frequency
pwm.pulse_width_percent(50)  # Set the pulse width to 50%

# Encoder setup
optical = ADC('A3')
gap_count = 0
last_state = 1  # Assume starting with no gap (sensor not blocked)
revs = 0
debounce_time = 0.05  # 50ms debounce period

def check_optical_sensor():
    value = optical.read()
    voltage = value * 3.3 / 4095
    if voltage > 0.5:
        return 0  # Gap detected
    else:
        return 1  # No gap

try:
    last_debounce_time = time.time()
    while True:
        current_state = check_optical_sensor()
        if time.time() - last_debounce_time > debounce_time:
            if current_state == 0 and last_state == 1:
                # Transition from no gap to gap
                gap_count += 1
                last_debounce_time = time.time()
                if gap_count >= 20:
                    revs += 1
                    gap_count = 0  # Reset after counting 20 gaps
                    # Stop motor after one revolution
                    if revs >= 1:
                        pwm.pulse_width_percent(0)  # Stop the motor
                        print(f"Completed {revs} revolution(s)")
                        break
                print(f"Gap {gap_count}")
            last_state = current_state  # Update the last state
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Stopping the motor")
    pwm.pulse_width_percent(0)  # Stop the motor
    # reset_mcu()  # Reset MCU if necessary

finally:
    # Ensure the motor is stopped
    pwm.pulse_width_percent(0)

