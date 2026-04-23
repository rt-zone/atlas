from machine import PWM, Pin, Timer
from .Encoder import Encoder
from math import pi
from .pid import PID

PWM_FREQ = 20_000
WHEEL_DIAMETER_CM = 4.5
WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER_CM
MAGIC_POWER_COEFFICIENT = 0.15
COUNTS_PER_REV = 357.75
PID_PERIOD = 0.02
DECEL_COUNTS = 500  # tune: increase if overshoot, decrease if brakes too early

HOLD_COEFFICIENT = 0.003
HOLD_INTEGRAL_COEFFICIENT = 0.01  # tune: increase if motor doesn't fully reach target
HOLD_INTEGRAL_MAX = 0.8             # tune: caps how much integral can contribute
HOLD_ERROR = 3

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def _find_k(travel, target_counts):
    # Ramp-up only — deceleration is handled in _calculate_cmd via DECEL_COUNTS
    md = 200
    k = 1.0
    if target_counts < md:
        ds = target_counts / 3
        if travel <= ds:
            k = travel / ds
    else:
        if travel <= md / 2:
            k = travel / (md / 2)
    return max(0.2, min(1.0, k))


class Motor:
    def __init__(self, motor_letter, velocity=1):
        if motor_letter == 'A':
            pin1, pin2 = "MOTOR_A1", "MOTOR_A2"
        elif motor_letter == 'B':
            pin1, pin2 = "MOTOR_B1", "MOTOR_B2"
        else:
            raise ValueError(f"Invalid motor letter '{motor_letter}'. Use 'A' or 'B'.")

        self.letter = motor_letter
        self.velocity = velocity
        self.m1 = PWM(Pin(pin1), freq=PWM_FREQ)
        self.m2 = PWM(Pin(pin2), freq=PWM_FREQ)
        self.encoder = Encoder(motor_letter)

        # pos_pid removed — speed profile is now shaped geometrically in _calculate_cmd
        # Kd=0 to avoid amplifying encoder noise
        self.vel_pid = PID(36, 0.001, 0.0, PID_PERIOD)

        self._hold_timer = Timer()
        self._move_seconds_timer = Timer()

        self._hold_integral = 0.0
    def _set_speed_raw(self, speed):
        speed = clamp(speed)
        duty = int(abs(speed) ** MAGIC_POWER_COEFFICIENT * 65535)
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

        k = _find_k(travel, target)

        # Ramp up: scale target speed by k at start of movement
        accel_speed = k * self.velocity

        # Ramp down: linearly reduce target speed as we approach the target
        decel_speed = clamp(remaining / DECEL_COUNTS, 0.2, self.velocity)

        # Whichever profile is more restrictive wins
        self.vel_pid.setpoint = min(accel_speed, decel_speed)

        # Use magnitude only — direction is applied externally by MotorController
        speed = abs(self.encoder.get_speed())
        accel = self.vel_pid.calculate(speed)

        increment = accel * PID_PERIOD
        if increment > 0:
            increment *= k * k  # only gate acceleration, never braking

        self.motor_cmd += increment
        self.motor_cmd = clamp(self.motor_cmd, 0, 1)
        return self.motor_cmd

    def _get_travel(self):
        return abs(self.encoder.get_count() - self.start_count)

    def _stop_all_timers(self):
        self._hold_timer.deinit()
        self._move_seconds_timer.deinit()

    def move_by_count_prepare(self, count):
        self._stop_all_timers()
        self.start_count = self.encoder.get_count()
        self.distance_count = count
        self.vel_pid.reset()
        self.vel_pid.setpoint = 0
        self.motor_cmd = 0.0

    def move_seconds(self, seconds, speed, on_complete=None):
        self._stop_all_timers()
        self.set_speed(speed)

        def _handler(timer):
            self.hold()
            timer.deinit()
            if on_complete:
                on_complete()

        self._move_seconds_timer.init(
            mode=Timer.ONE_SHOT,
            period=int(seconds * 1000),
            callback=_handler
        )

    def stop(self):
        self._stop_all_timers()  # Fixed: was missing ()
        self.set_speed(0)

    def hold(self, hold_position=None):
        self.hold_position = hold_position if hold_position is not None else self.encoder.get_count()
        self._hold_integral = 0.0  # reset integral every time hold is re-armed
        self._hold_timer.init(mode=Timer.PERIODIC, freq=int(1 / PID_PERIOD), callback=self._hold_handler)

    def _hold_handler(self, timer):
        distance = self.hold_position - self.encoder.get_count()

        if abs(distance) < HOLD_ERROR:
            self._hold_integral = 0.0  # clear integral when settled, prevents accumulated windup on next disturbance
            self.set_speed(0)
            return

        self._hold_integral += distance * PID_PERIOD
        self._hold_integral = clamp(self._hold_integral, -HOLD_INTEGRAL_MAX, HOLD_INTEGRAL_MAX)

        p = distance * HOLD_COEFFICIENT
        i = self._hold_integral * HOLD_INTEGRAL_COEFFICIENT

        self.set_speed(p + i)
    def release(self):
        self._hold_timer.deinit()