from machine import PWM, Pin

PWM_FREQ = 20_000

class Motor:
    def __init__(self, motor):
        pin1, pin2 = None, None
        if motor == 'A':
            pin1 = "MOTOR_A1"
            pin2 = "MOTOR_A2"
        elif motor == 'B':
            pin1 = "MOTOR_B1"
            pin2 = "MOTOR_B2"
        else:
            raise ValueError(f"Invalid motor letter'{motor}'. Use 'A' or 'B'.")
        
        self.motor = motor
        self.m1 = PWM(Pin(pin1), freq=PWM_FREQ)
        self.m2 = PWM(Pin(pin2), freq=PWM_FREQ)

    def set_speed(self, speed):
        # speed expected -1..1, but original code used 0..1 only for forward
        if speed >= 0:
            self.m1.duty_u16(int(max(0, speed) * 65535))
            self.m2.duty_u16(0)
        else:
            self.m1.duty_u16(0)
            self.m2.duty_u16(int(max(0, -speed) * 65535))

    def moveMotorDegrees(self, motor, distance_deg):
        direction = 1 if distance_deg >= 0 else -1
        distance_deg = abs(distance_deg)

        if motor == 'A':
            start_left = encoders('A').capture().degrees
            self.set_motor(self.left_in1, self.left_in2, self.LeftSpeed * direction)

            try: 
                while True:
                    left_cap  = encoders('A').capture().degrees
                    left_travel = abs(left_cap - start_left)
                    if (left_travel >= distance_deg):
                        break
            finally:
                self.set_motor(self.left_in1, self.left_in2, 0)

        else: 
            start_right = encoders('B').capture().degrees
            self.set_motor(self.right_in1, self.right_in2, self.RightSpeed * direction)
            
            try: 
                while True:
                    right_cap = encoders('B').capture().degrees
                    right_travel = abs(right_cap - start_right)
                    if (right_travel >= distance_deg):
                        break
            finally:
                self.set_motor(self.left_in1, self.left_in2, 0)