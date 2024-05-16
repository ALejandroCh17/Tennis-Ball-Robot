import time
from robot_hat import PWM, ADC

# Setup
pwm_channel = 'P0'
pwm = PWM(pwm_channel)
pwm.freq(800)
pwm.pulse_width_percent(0)  # Ensure motor starts stopped
optical = ADC('A3')
ir_sensor = ADC('A0')

def check_ir_sensor():
    value = ir_sensor.read()
    voltage = value * 3.3 / 4095
    if voltage > 0.5:
        return True
    return False


try:
    while True:
        if not check_ir_sensor():
            pwm.pulse_width_percent(35)  # Activate motor in unsafe state
            time.sleep(0.028)
            pwm.pulse_width_percent(0)
        else:
            pwm.pulse_width_percent(35)
            time.sleep(0.028 * 3)
            pwm.pulse_width_percent(0)  # Deactivate motor in safe state
            break 

        time.sleep(0.09)  # Small delay to reduce processing load
except KeyboardInterrupt:
    pwm.pulse_width_percent(0)  # Ensure motor is stopped when script is interrupted

