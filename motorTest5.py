from robot_hat import Motors, reset_mcu
import time

motors = Motors()

motors.set_left_id(1)
motors.set_right_id(2)

runtime = True

while runtime:
    motors.forward(100)
    time.sleep(30)
    runtime = False

motors.stop()
time.sleep(1)
runtime = True

while runtime:
    motors.turn_left(30)
    time.sleep(3)
    runtime = False

motors.stop()
time.sleep(5)
runtime=True
while runtime:
    motors.speed(-30,30)
    time.sleep(7)
    runtime=False

motors.stop()
