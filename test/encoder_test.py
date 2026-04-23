from atlas import mc 


while True:
    enca = mc.motor_a.encoder.get_count()
    encb = mc.motor_b.encoder.get_count()
    print(enca, encb)