from .Motor import Motor
from .Encoder import Encoder
import time 
from PID import PID 


MAGIC_TURN_CONVERTER = 1.79


def _find_k_turn(travel):
    mds = 50
    k = 1
    if travel <= mds:
        k = (travel / mds) * 0.4
    return max(0.1, min(0.4, k))


class MotorsController:
    def __init__(self):
        self.motor_a = Motor('A')
        self.motor_b = Motor('B')


        self.dt = 0.02
        self.left_pos_pid  = PID(0.005, 0.0, 0, self.dt)
        self.right_pos_pid = PID(0.005, 0.0, 0, self.dt)
        self.left_vel_pid  = PID(10, 0.0, 0.001, self.dt)
        self.right_vel_pid = PID(10, 0.0, 0.001, self.dt)
        self.align_pid = PID(0, 0.0, 0, self.dt)

    def _get_motor(self, motor) -> Motor:
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
    
    
    def turn(self, turn_degrees):
        direction =  1 if turn_degrees > 0 else -1
        turn_degrees = abs(turn_degrees)

        destination_degrees = turn_degrees * MAGIC_TURN_CONVERTER

        # Lookup table for degrees_to_stop based on turn_degrees
        lookup_table = [
            (360, 62),
            (270, 70),
            (180, 70),
            (90, 74),
            (80, 70),
            (70, 60),
            (60, 50),
            (45, 25),
        ]
        
        degrees_to_stop = next(
            (value for threshold, value in lookup_table if turn_degrees >= threshold),
            turn_degrees / 2
        )

        start_left = self.motor_a.encoder.get_degrees()
        start_right = self.motor_b.encoder.get_degrees()


        self.left_pos_pid.setpoint = destination_degrees
        self.right_pos_pid.setpoint = destination_degrees

        last_left_cmd = 0.0
        last_right_cmd = 0.0

        self._moving = True
        while True:
            if self._stop:
                self._stop = False
                return

            left_cap  = self.motor_a.encoder.capture()
            right_cap = self.motor_b.encoder.capture()

            left_travel = (left_cap.degrees - start_left) * direction
            right_travel = (right_cap.degrees - start_right) * (-direction)

            travel = (left_travel + right_travel) / 2

            left_speed = (left_cap.revolutions_per_second * direction) / 4
            right_speed = (right_cap.revolutions_per_second * (-direction)) / 4

            if (travel >= destination_degrees - degrees_to_stop):
                break

            left_vel = self.left_pos_pid.calculate(left_travel)
            right_vel = self.right_pos_pid.calculate(right_travel)

            k = _find_k_turn(travel)

            self.left_vel_pid.setpoint = max(min(left_vel, k), -k)
            self.right_vel_pid.setpoint = max(min(right_vel, k), -k)

            left_accel = self.left_vel_pid.calculate(left_speed)
            right_accel = self.right_vel_pid.calculate(right_speed)

            last_left_cmd += left_accel * self.dt
            last_right_cmd += right_accel * self.dt

            speed_error = left_travel - right_travel
            correction = self.align_pid.calculate(speed_error)

            last_left_cmd  += correction
            last_right_cmd -= correction

            last_left_cmd = max(min(last_left_cmd, 1), 0)
            last_right_cmd = max(min(last_right_cmd, 1), 0)

            final_left = last_left_cmd * direction
            final_right = last_right_cmd * (-direction)
            self.move(final_left, final_right)

            time.sleep(self.dt)

