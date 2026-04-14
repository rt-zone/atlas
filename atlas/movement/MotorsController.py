from .Motor import Motor
from .Encoders import encoders

import time 

def speed2power(speed):
    direction = 1 if speed > 0 else -1
    return (abs(speed) ** 0.15 * direction)

class MotorsController:
    def __init__(self):
        self.motor_a = Motor('A')
        self.motor_b = Motor('B')

    def __call__(self, motor):
        return self._get_motor(motor)

    def _get_motor(self, motor):
        if motor == 'A':
            return self.motor_a
        elif motor == 'B':
            return self.motor_b
        else:
            raise ValueError(f"Invalid motor letter'{motor}'. Use 'A' or 'B'.")


    def move(self, left_speed, right_speed):
        """ left_speed/right_speed between -1 and 1 """
        self.motor_a.set_speed(left_speed)
        self.motor_b.set_speed(right_speed)
    
    def _move(self, left_speed, right_speed):
        # left_speed/right_speed between -1 and 1
        left_power = speed2power(left_speed)
        right_power = speed2power(right_speed)
        
        self.motor_a.set_speed(left_power)
        self.motor_b.set_speed(right_power)

    def _stop_smooth(self):
        left_speed  = encoders('A').revolutions_per_second / 4
        right_speed = encoders('B').revolutions_per_second / 4


        l = -1
        r = -1
        if left_speed <= 0:
            l = 1
        if right_speed <= 0:
            r = 1

        sleep_time = abs(left_speed + right_speed) / 40
        self.move(l,r)
        time.sleep(sleep_time)
        self.move(0,0)
        time.sleep(0.5)
        self._moving = False
        print("smooth stop")


    
    def move_motor(self, motor_letter, distance_deg, speed):
        direction = 1 if distance_deg >= 0 else -1
        distance_deg = abs(distance_deg)
        motor = self(motor_letter)

        motor.set_speed(speed * direction)
        enc = encoders(motor_letter)
        start_degrees = enc.get_degrees()
        # TODO: make it non blocking using interrupt
        while abs(enc.get_degrees() - start_degrees) < distance_deg:
            pass

        self.motor_a.set_speed(0)


    def move_motor_degrees(self, motor, distance_deg):
        if motor == 'A':
            self.move_motor(motor, distance_deg, self.LeftSpeed)
        else: 
            self.move_motor(motor, distance_deg, self.RightSpeed)
            