from robot_hat import PWM

p0 = PWM('P0')

p0.freq(1000)
p0.pulse_width_percent(50)

while True:
    continue


