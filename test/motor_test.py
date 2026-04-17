from atlas.movement.Motor import Motor
ma = Motor('A')
mb = Motor('B')

def print_info(motor : Motor):
    print(motor.letter, motor.encoder.get_revolutions(), motor.encoder.get_count())

ma.move_cm(100, lambda: print_info(ma))
mb.move_cm(100, lambda: print_info(mb))

from time import sleep
sleep(5)

ma.move_cm(-100, lambda: print_info(ma))
mb.move_cm(-100, lambda: print_info(mb))


try:
    max_speed = 0
    while True:
        speed = abs(ma.encoder.get_speed())
        max_speed = max(max_speed, speed)
        pass
finally:
    print(max_speed)
    ma.stop()   
    ma.release()
    mb.stop()
    mb.release()