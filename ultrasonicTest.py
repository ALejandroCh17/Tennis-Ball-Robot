# Import Ultrasonic and Pin class
from robot_hat import Ultrasonic, Pin
import time

# Create Motor object
us = Ultrasonic(Pin("D0"), Pin("D1"))

# Read distance
while True:
    distance = us.read()
    print(f"Distance: {distance}cm")
    time.sleep(.35)
