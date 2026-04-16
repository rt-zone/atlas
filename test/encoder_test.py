from atlas.movement.Encoder import Encoder
from machine import Pin

m1 = Pin("MOTOR_A1", Pin.OUT)
m2 = Pin("MOTOR_A2", Pin.OUT)

encoder = Encoder('A')

def m1_handler(pin):
    m1.value(pin.value())

def m2_handler(pin):
    m2.value(pin.value())


Pin("ENC_A1").irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=m1_handler)
Pin("ENC_A2").irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=m2_handler)

while True:
    # print(encoder.get_speed())
    pass