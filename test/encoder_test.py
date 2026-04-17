
from atlas.movement.Motor import Motor

ma = Motor('A')
mb = Motor('B')

print(ma.encoder.get_count())
while True:
    print(ma.encoder.get_count())