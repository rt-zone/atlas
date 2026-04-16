from atlas.movement.Motor import Motor, WHEEL_CIRCUMFERENCE
ma = Motor('A')
mb = Motor('B')

def print_count():
    print(ma.encoder.get_rotations() * WHEEL_CIRCUMFERENCE)

print_count()
ma.move_by_cm(50, on_complete=print_count)
mb.move_by_cm(50)


try:
    while True:
        pass
finally:
    ma.stop()
    ma.release()
    mb.stop()
    mb.release()