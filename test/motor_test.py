from atlas.movement.MotorController import MotorController
from time import sleep
mc = MotorController()
mc.move(1000)
sleep(3)
mc.move(-1000)
sleep(3)
mc.move(3000)
sleep(4)
mc.move(-3000)
# from atlas.movement.Motor import Motor


# ma = Motor("A")
# mb = Motor("B")

# ma._move_by_counts(1000)
# mb._move_by_counts(1000)
# ma.set_speed(1)
# mb.set_speed(1)

while True:
    print(mc.motor_a.encoder.get_count(), mc.motor_b.encoder.get_count())
    pass    