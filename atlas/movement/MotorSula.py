from machine import PWM, Pin
from .Encoder import Encoder
import time
from math import pi
from PID import PID

PWM_FREQ = 20_000
WHEEL_DIAMETER_CM = 4.5
WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER_CM

def _speed2power(speed):
    direction = 1 if speed > 0 else -1
    return abs(speed) ** 0.15 * direction

def _find_k(travel, degrees_need):
    mds = 100
    mdt = 100
    k = 1
    if degrees_need < (mds + mdt):
        ds = degrees_need / 3
        if travel <= ds:
            k = travel / ds
    else:
        if travel <= mds:
            k = travel / mds
    return max(0.2, min(1, k))


class Motor:
    def __init__(self, motor_letter, velocity=1.0):
        if motor_letter == 'A':
            pin1, pin2 = "MOTOR_A1", "MOTOR_A2"
        elif motor_letter == 'B':
            pin1, pin2 = "MOTOR_B1", "MOTOR_B2"
        else:
            raise ValueError(f"Invalid motor letter '{motor_letter}'.")

        self.motor    = motor_letter
        self.velocity = velocity
        self.m1 = PWM(Pin(pin1), freq=PWM_FREQ)
        self.m2 = PWM(Pin(pin2), freq=PWM_FREQ)
        self.encoder  = Encoder(motor_letter)

        self.dt = 0.02

        # same values as his _f PIDs
        self.pos_pid = PID(0.002, 0.0, 0,      self.dt)
        self.vel_pid = PID(36,    0.0, 0.0005, self.dt)

    def _set_motor(self, speed):
        speed = max(-1.0, min(1.0, speed))
        power = _speed2power(speed)
        duty  = int(abs(power) * 65535)
        if power >= 0:
            self.m1.duty_u16(0)
            self.m2.duty_u16(duty)
        else:
            self.m1.duty_u16(duty)
            self.m2.duty_u16(0)

    def stop(self):
        self.m1.duty_u16(0)
        self.m2.duty_u16(0)
        
    

    def move_by_cm(self, distance_cm):
        direction      = 1 if distance_cm >= 0 else -1
        distance_cm    = abs(distance_cm)
        distance_deg   = (distance_cm / WHEEL_CIRCUMFERENCE) * 360 + 90   # +90 same as his code

        start = self.encoder.capture().degrees

        self.pos_pid.setpoint = distance_deg
        self.vel_pid.setpoint = 0

        last_cmd = 0.0


        while True:
            cap     = self.encoder.capture()
            travel  = abs(cap.degrees - start)
            speed   = abs(cap.revolutions_per_second) / 4   # normalised 0..1

            if travel >= distance_deg - 90:                  # -90 same as his break condition
                break

            k = _find_k(travel, distance_deg)

            target_vel = self.pos_pid.calculate(travel)
            self.vel_pid.setpoint = max(min(target_vel, self.velocity), -self.velocity)

            accel    = self.vel_pid.calculate(speed)
            last_cmd += k * k * accel * self.dt
            last_cmd  = max(min(last_cmd, self.velocity), 0)

            self._set_motor(last_cmd * direction)
            time.sleep(self.dt)

        self.stop()

    def move_by_degrees(self, degrees):
        cm = (abs(degrees) / 360) * WHEEL_CIRCUMFERENCE
        self.move_by_cm(cm if degrees >= 0 else -cm)

    def move_by_rotations(self, rotations):
        self.move_by_cm(rotations * WHEEL_CIRCUMFERENCE)