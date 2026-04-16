from machine import PWM, Pin, Timer
from .Encoder import Encoder
import time
from math import pi
from PID import PID

PWM_FREQ = 100_000
WHEEL_DIAMETER_CM = 4.5
WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER_CM

PID_FREQ    = 50            # Hz
PID_PERIOD  = 1.0 / PID_FREQ
MAX_RPS     = 4.0           # from Encoder module

MAGIC_POWER_COEFFICIENT = 0.15

# Velocity PID — controls speed during movement
VEL_KP = 30.0
VEL_KI = 0.0
VEL_KD = 0.4

# Hold — simple P controller on encoder counts
HOLD_KP         = 0.02
HOLD_DEADBAND   = 3     # counts, avoids jitter


class Motor:
    _IDLE    = 0
    _MOVING  = 1
    _HOLDING = 2

    def __init__(self, motor_letter, velocity=1.0):
        if motor_letter == 'A':
            pin1, pin2 = "MOTOR_A1", "MOTOR_A2"
        elif motor_letter == 'B':
            pin1, pin2 = "MOTOR_B1", "MOTOR_B2"
        else:
            raise ValueError(f"Invalid motor letter '{motor_letter}'. Use 'A' or 'B'.")

        self.motor    = motor_letter
        self.velocity = velocity
        self.m1 = PWM(Pin(pin1), freq=PWM_FREQ)
        self.m2 = PWM(Pin(pin2), freq=PWM_FREQ)
        self.encoder  = Encoder(motor_letter)
        self.vel_pid  = PID(VEL_KP, VEL_KI, VEL_KD, PID_PERIOD)

        self._state          = Motor._IDLE
        self._timer          = Timer()
        self._direction      = 1
        self._start_degrees  = 0.0
        self._target_degrees = 0.0
        self._output         = 0.0      # integrated motor speed
        self._hold_count     = 0
        self._hold_after     = True
        self.on_complete     = None

    # ------------------------------------------------------------------ #
    #  Core drive                                                          #
    # ------------------------------------------------------------------ #

    def _move_raw(self, speed):
        speed = max(-1.0, min(1.0, speed))
        if abs(speed) < 0.001:
            self.m1.duty_u16(0)
            self.m2.duty_u16(0)
            return
        duty = int((abs(speed) ** MAGIC_POWER_COEFFICIENT) * 65535)
        if speed >= 0:
            self.m1.duty_u16(0)
            self.m2.duty_u16(duty)
        else:
            self.m1.duty_u16(duty)
            self.m2.duty_u16(0)

    def move(self, speed):
        self._move_raw(speed * self.velocity)

    # ------------------------------------------------------------------ #
    #  Speed ramp                                                          #
    # ------------------------------------------------------------------ #

    def _speed_profile(self, travelled, total):
        """
        Trapezoidal ramp:
          - ramp up over first 30% (or 120 deg, whichever is smaller)
          - cruise at full speed
          - ramp down symmetrically at the end
        Returns a k in [0.15, 1.0]
        """
        ramp = min(total * 0.3, 120.0)
        remaining = total - travelled

        if travelled < ramp:
            k = travelled / ramp
        elif remaining < ramp:
            k = remaining / ramp
        else:
            k = 1.0

        return max(0.15, min(1.0, k))

    # ------------------------------------------------------------------ #
    #  Timer callback — runs at PID_FREQ                                  #
    # ------------------------------------------------------------------ #

    def _pid_tick(self, timer):
        capture = self.encoder.capture()

        if self._state == Motor._MOVING:
            travelled = abs(capture.degrees - self._start_degrees)

            if travelled >= self._target_degrees:
                self._move_raw(0)
                self._output = 0.0
                if self._hold_after:
                    self._hold_count = capture.count
                    self._state = Motor._HOLDING
                else:
                    self._state = Motor._IDLE
                    self._timer.deinit()
                if self.on_complete:
                    self.on_complete()
                return

            # Desired normalised RPS scaled by ramp profile
            k = self._speed_profile(travelled, self._target_degrees)
            self.vel_pid.setpoint = k  # target is fraction of MAX_RPS (0..1)

            actual_rps_norm = capture.revolutions_per_second / MAX_RPS
            accel = self.vel_pid.calculate(actual_rps_norm)

            # Integrate acceleration into output speed, clamp to ±1
            self._output = max(-1.0, min(1.0, self._output + accel * PID_PERIOD))
            self._move_raw(self._output * self._direction)

        elif self._state == Motor._HOLDING:
            error = self._hold_count - capture.count
            if abs(error) <= HOLD_DEADBAND:
                self._move_raw(0)
            else:
                # Cap correction to 40% speed so it doesn't slam
                correction = max(-0.4, min(error * HOLD_KP, 0.4))
                self._move_raw(correction)

    # ------------------------------------------------------------------ #
    #  Public movement API                                                 #
    # ------------------------------------------------------------------ #

    def move_by_degrees(self, degrees, hold=True, on_complete=None):
        """Non-blocking. Moves by degrees then optionally holds position."""
        self._timer.deinit()
        self.vel_pid.setpoint = 0       # reset integrator
        self._output         = 0.0
        self._direction      = 1 if degrees >= 0 else -1
        self._target_degrees = abs(degrees)
        self._start_degrees  = self.encoder.get_degrees()
        self._hold_after     = hold
        self.on_complete     = on_complete
        self._state          = Motor._MOVING
        self._timer.init(freq=PID_FREQ, mode=Timer.PERIODIC, callback=self._pid_tick)

    def move_by_rotations(self, rotations, hold=True, on_complete=None):
        self.move_by_degrees(rotations * 360, hold, on_complete)

    def move_by_cm(self, centimeters, hold=True, on_complete=None):
        self.move_by_rotations(centimeters / WHEEL_CIRCUMFERENCE, hold, on_complete)

    def move_by_seconds(self, seconds, speed):
        self.move(speed)
        time.sleep(seconds)
        self.stop()

    def hold(self):
        """Hold current position immediately."""
        self._timer.deinit()
        self._hold_count = self.encoder.get_count()
        self._state      = Motor._HOLDING
        self._timer.init(freq=PID_FREQ, mode=Timer.PERIODIC, callback=self._pid_tick)

    def stop(self):
        """Stop and coast — no hold."""
        self._timer.deinit()
        self._state = Motor._IDLE
        self._move_raw(0)

    def release(self):
        """Alias for stop — releases hold if active."""
        self.stop()