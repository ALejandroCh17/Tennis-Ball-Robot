from robot_hat import Motors
import time

motors = Motors()

motors.set_left_id(1)
motors.set_right_id(2)

#motors.forward(70)
motors.speed(50,0)
time.sleep(4)
motors.stop()
