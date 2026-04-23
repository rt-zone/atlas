from atlas import MotorController
from time import sleep

mc = MotorController()

sleep(1)

# mc.turn_by_degrees(3600)

for i in range(4):
    mc.move_cm(100)
    mc.turn_by_degrees(90)