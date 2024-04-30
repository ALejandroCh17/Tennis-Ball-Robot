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

gap_count = 0
last_state = 1

def check_optical_sensor():
    value = optical.read()
    voltage = value * 3.3 / 4095
    if voltage > 0.5:
        return 0  # Gap detected
    else:
        return 1  # No gap


while(True):
    print(gap_count)
    pwm.freq(800)
    pwm.pulse_width_percent(50)
    current_state = check_optical_sensor()
    if current_state == 0 and last_state == 1:
        # Transition from no gap to gap
        gap_count += 1
        if gap_count > 12:
            revs += 1
            pwm.pulse_width_percent(0)
            break
    last_state = current_state
    time.sleep(0.01)
print(gap_count)    

    
