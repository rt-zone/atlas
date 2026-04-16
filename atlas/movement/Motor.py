from machine import PWM, Pin, Timer
from .Encoder import Encoder
import time
from math import pi
from PID import PID

PWM_FREQ = 20_000
WHEEL_DIAMETER_CM = 4.5
WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER_CM
HOLD_COEFFICIENT = 0.003
HOLD_ERROR = 2
MAGIC_POWER_COEFFICIENT = 0.15

PID_PERIOD = 0.02 # 50Hz
FEED_FORWARD = 0.3

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


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
    def __init__(self, motor_letter, velocity=1):
        pin1, pin2 = None, None
        if motor_letter == 'A':
            pin1 = "MOTOR_A1"
            pin2 = "MOTOR_A2"
        elif motor_letter == 'B':
            pin1 = "MOTOR_B1"
            pin2 = "MOTOR_B2"
        else:
            raise ValueError(f"Invalid motor letter'{motor_letter}'. Use 'A' or 'B'.")
        
        self.motor = motor_letter
        self.velocity = velocity
        self.m1 = PWM(Pin(pin1), freq=PWM_FREQ)
        self.m2 = PWM(Pin(pin2), freq=PWM_FREQ)
        self.encoder = Encoder(motor_letter)
        
        self.pos_pid = PID(0.002, 0.0, 0,      PID_PERIOD)
        self.vel_pid = PID(36,    0.0, 0.0005, PID_PERIOD)

        self._timer = Timer()
    

    def _move_raw(self, speed):
        # speed expected -1..1, but original code used 0..1 only for forward
        speed = max(-1, min(speed, 1))
        duty = abs(speed) ** MAGIC_POWER_COEFFICIENT
        duty = int(duty * 65535)

        if speed >= 0:
            self.m1.duty_u16(duty)
            self.m2.duty_u16(0)
        else:
            self.m1.duty_u16(0)
            self.m2.duty_u16(duty)



    def move(self, speed):
        self._move_raw(speed * self.velocity)        

    def _calculate_increase_in_cmd(self, travel, target):
        k = _find_k(travel, target)
        speed = self.encoder.get_speed()
        target_speed = self.pos_pid.calculate(travel)
        self.vel_pid.setpoint = clamp(target_speed, -self.velocity, self.velocity)
        accel = self.vel_pid.calculate(speed)
        
        return k * k * accel * PID_PERIOD

    def _on_finish(self, on_complete):
        self.hold()
        self._timer.deinit()
        if on_complete:
            on_complete()

    def _move_cm_tick(self, direction, on_complete):
        travel  = abs(self.encoder.get_degrees() - self.start_degrees)

        if travel >= self.target_degrees - 90:                  # -90 same as his break condition
            self._on_finish(on_complete)
            return

        i = self._calculate_increase_in_cmd(travel, self.target_degrees)
        self.motor_cmd += i
        self.motor_cmd = clamp(self.motor_cmd, 0, self.velocity)
        self.move(self.motor_cmd * direction)

    def move_by_cm(self, distance_cm, on_complete=None):
        direction      = 1 if distance_cm >= 0 else -1
        distance_cm    = abs(distance_cm)
        target_degrees   = (distance_cm / WHEEL_CIRCUMFERENCE) * 360 

        self.start_degrees = self.encoder.get_degrees() 
        self.target_degrees = target_degrees

        self.pos_pid.setpoint = target_degrees
        self.vel_pid.setpoint = 0

        self.motor_cmd = 0.0

        self._timer.init(mode=Timer.PERIODIC, freq=int(1/PID_PERIOD), callback=lambda t: self._move_cm_tick(direction, on_complete))

    
    # def move_by_degrees(self, destination_degree):
    #     self.target_degrees = abs(destination_degree)
    #     self.start_degrees = self.encoder.get_degrees()
    #     direction = 1 if destination_degree >= 0 else -1
    #     self.move(direction)
    #     self.encoder.append_irq(self._move_degrees_handler)

    # def _move_degrees_handler(self, pin):
    #     current_degrees = self.encoder.get_degrees() - self.start_degrees
    #     if abs(current_degrees) >= self.target_degrees:
    #         print("Stop")
    #         self.move(0)
    #         self.hold()
    #         self.encoder.remove_irq(self._move_degrees_handler)


    # def move_by_rotations(self, distance_rev):
    #     distance_deg = distance_rev * 360
    #     self.move_by_degrees(distance_deg)

    # def move_by_cm(self, centimeters):
    #     revolutions = centimeters / WHEEL_CIRCUMFERENCE
    #     self.move_by_rotations(revolutions)

    # def move_by_seconds(self, seconds, speed):
    #     self.move(speed)
    #     time.sleep(seconds)
    #     self.stop()

    def stop(self):
        self.move(0)

    def _hold_handler(self, pin):
        distance = self.current_hold_position - self.encoder.get_count()
        if abs(distance) < HOLD_ERROR:
            self.move(0)
            return
        self.move(distance * HOLD_COEFFICIENT)
    
    # def hold(self):
    #     self.current_hold_position = self.encoder.get_count()
    #     self.encoder.append_irq(self._hold_handler)

    # def release(self):
    #     self.encoder.remove_irq(self._hold_handler)

# Timer based hold function
    def hold(self):
        self.current_hold_position = self.encoder.get_count()
        self._hold_timer = Timer()
        self._hold_timer.init(mode=Timer.PERIODIC, freq=100, callback=self._hold_handler)

    def release(self):
        self._hold_timer.deinit()