from robot_hat import Pin, ADC
import time

optical = ADC('A3')
gap_count = 0
last_state = 1  # Assume starting with no gap (sensor not blocked)

def check_optical_sensor():
    value = optical.read()
    voltage = value * 3.3 / 4095
    if voltage > 0.5:
        return 0  # Gap detected
    else:
        return 1  # No gap

revs = 0

while True:
    current_state = check_optical_sensor()
    if current_state == 0 and last_state == 1:
        # Transition from no gap to gap
        gap_count += 1
        if gap_count > 20:
            revs += 1
            gap_count = 1  # Reset after counting 20 gaps
        print(f"Gap {gap_count}")
        print(revs)
    last_state = current_state  # Update the last state
    time.sleep(0.01)
