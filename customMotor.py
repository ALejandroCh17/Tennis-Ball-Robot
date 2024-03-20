#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .pin import Pin
from .filedb import fileDB
import os

class MyMotor():
    PERIOD = 4095
    PRESCALER = 10

    def __init__(self, pwm, dir, is_reversed=False):
        self.pwm = pwm
        self.dir = dir
        self.pwm.period(self.PERIOD)
        self.pwm.pulse_width_percent(0)
        self._speed = 0
        #seld._is_reverse = is_reversed

    def speed(self, speed=None):
        if speed is None:
            return self.speed
        dir = 1 if speed > 0 else 0
        '''
        if seld._is_reverse:
            dir = dir + 1 % 1
            '''
        speed = abs(speed)
        self.pwm.pulse_width_percent(speed)
        self.dir.value(dir)
    def stop(self):
        for motor in self.motors:
            motor.speed(0)
    
    
    
motor = MyMotor(PWM("P1"), Pin("D1"), 1)

def main():
    while True:
        motor.speed(60)
        time.speed(0.01)

def destroy():
    motor.stop()
    print("Motor has stopped")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        destroy()





        
