from machine import PWM, Pin, Timer
from .Encoder import Encoder
from math import pi
from .pid import PID

PWM_FREQ = 20_000
HOLD_COEFFICIENT = 0.003
HOLD_ERROR = 3
MAGIC_POWER_COEFFICIENT = 0.15
PID_PERIOD = 0.01
DECEL_COUNTS = 2000

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))

def _find_k(travel, target_counts):
    md = 400
    k = 1.0

    if target_counts < md:
        ds = target_counts / 3
        if travel <= ds:
            k = travel / ds
    else:
        if travel <= md / 2:
            k = travel / (md / 2)

    return max(0.2, min(1.0, k))

# TODO: Add timer manager like in buzzer
class Motor:
    def __init__(self, motor_letter, velocity=1.0):
        pin1, pin2 = None, None
        if motor_letter == 'A':
            pin1 = "MOTOR_A1"
            pin2 = "MOTOR_A2"
        elif motor_letter == 'B':
            pin1 = "MOTOR_B1"
            pin2 = "MOTOR_B2"
        else:
            raise ValueError(f"Invalid motor letter'{motor_letter}'. Use 'A' or 'B'.")
        
        self.letter = motor_letter
        self.velocity = velocity
        self.m1 = PWM(Pin(pin1), freq=PWM_FREQ)
        self.m2 = PWM(Pin(pin2), freq=PWM_FREQ)
        self.encoder = Encoder(motor_letter)
        
        self.pos_pid = PID(0.001, 0.001, 0.001,PID_PERIOD)
        self.vel_pid = PID(30,    0.001, 0.01, PID_PERIOD)

        self._move_pid_timer = Timer()
        self._hold_timer = Timer()
        self._move_seconds_timer = Timer()

    def _set_speed_raw(self, speed):
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


    def set_speed(self, speed):
        self._set_speed_raw(speed * self.velocity)        


    def _calculate_cmd(self):
        travel = self._get_travel()
        target = abs(self.distance_count)
        remaining = target - travel

        # Ramp up based on how far we've traveled
        k = _find_k(travel, target) * self.velocity
        accel_speed = k
        # Ramp down based on how far we have left
        decel_speed = clamp(remaining / DECEL_COUNTS, 0.05, self.velocity)

        # Whichever profile is more restrictive wins
        self.vel_pid.setpoint = min(accel_speed, decel_speed)

        # Always compare against magnitude — direction is handled externally
        speed = abs(self.encoder.get_speed())
        accel = self.vel_pid.calculate(speed)

        increment = k*k*accel * PID_PERIOD

        self.motor_cmd += increment
        self.motor_cmd = clamp(self.motor_cmd, 0, 1)
        return self.motor_cmd
    def _get_travel(self):
        return abs(self.encoder.get_count() - self.start_count)

    def _stop_all_timers(self):
        self._hold_timer.deinit()
        self._move_pid_timer.deinit()
        self._move_seconds_timer.deinit()
    
    def move_by_count_prepare(self, count):
        self._stop_all_timers()
        self.start_count = self.encoder.get_count()
        self.distance_count = count
        self.pos_pid.reset()
        self.vel_pid.reset()
        self.pos_pid.setpoint = abs(count)
        self.vel_pid.setpoint = 0
        self.motor_cmd = 0.0
    
    def stop(self):
        self._stop_all_timers()
        self.set_speed(0)

    def _hold_handler(self, timer):
        distance = self.hold_position - self.encoder.get_count()
        if abs(distance) < HOLD_ERROR:
            self.set_speed(0)
            return
        self.set_speed(distance * HOLD_COEFFICIENT)

# Timer based hold function
    def hold(self, hold_position = None):
        if hold_position is not None:
            self.hold_position = hold_position
        else:
            self.hold_position = self.encoder.get_count()

        self._hold_timer.init(mode=Timer.PERIODIC, freq=int(1/PID_PERIOD), callback=self._hold_handler)

    def release(self):
        self._hold_timer.deinit()





    
    def move_revolutions(self, distance_rev, on_complete=None):
        target_count = int(distance_rev * COUNTS_PER_REV)
        self._move_by_counts(target_count, on_complete)

    def move_degrees(self, distance_degree, on_complete=None):
        target_revolutions = distance_degree / 360
        self.move_revolutions(target_revolutions, on_complete)

    def move_cm(self, centimeters, on_complete=None):
        revolutions = centimeters / WHEEL_CIRCUMFERENCE
        self.move_revolutions(revolutions, on_complete)

    def move_seconds(self, seconds, speed, on_complete=None):
        self._hold_timer.deinit()
        self._move_pid_timer.deinit()
        self._move_seconds_timer.deinit()
        self.set_speed(speed)
        def move_by_seconds_handler(timer):
            if on_complete:
                on_complete()
            self.hold()
            timer.deinit()


        # Store it so gc wouldn't collect it 
        self._move_seconds_timer.init(
            mode=Timer.ONE_SHOT, 
            period=int(seconds * 1000), 
            callback=move_by_seconds_handler
            )

