from machine import Timer
from .Motor import Motor
from .pid import PID
from math import pi
from time import sleep

SYNC_PID_PERIOD = 0.01
COUNTS_PER_REV = 357.75
WHEEL_DIAMETER_CM = 4.5
WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER_CM

AFTER_MOVE_DELAY = 0.5
TURN_COEFFICIENT = 1.78

class MotorController:
    def __init__(self, velocity=1.0):
        self.motor_a = Motor('A', velocity)
        self.motor_b = Motor('B', velocity)

        self.sync_pid = PID(0.02, 0.001, 0.0, SYNC_PID_PERIOD)
        self.sync_pid.setpoint = 0  # we always want zero drift between motors

        self._sync_timer = Timer()
        self._is_moving = False

    def _start_move(self, count_a, count_b, is_blocking, on_complete):
        self._is_moving = True
        self._sync_timer.deinit()
        dir_a = 1 if count_a >= 0 else -1
        dir_b = 1 if count_b >= 0 else -1

        self.motor_a.move_by_count_prepare(count_a)
        self.motor_b.move_by_count_prepare(count_b)
        self.sync_pid.reset()


        self._sync_timer.init(
            mode=Timer.PERIODIC,
            freq=int(1 / SYNC_PID_PERIOD),
            callback=lambda t: self._sync_tick(dir_a, dir_b, on_complete, t)
        )

        if is_blocking:
            while self._is_moving:
                pass
            sleep(AFTER_MOVE_DELAY)


    def _sync_tick(self, dir_a, dir_b, on_complete, timer):
        travel_a = self.motor_a._get_travel()
        travel_b = self.motor_b._get_travel()

        target_a = abs(self.motor_a.distance_count)
        target_b = abs(self.motor_b.distance_count)

        done_a = travel_a >= target_a
        done_b = travel_b >= target_b

        if done_a and done_b:
            self.hold()
            timer.deinit()
            self._is_moving = False
            if on_complete:
                on_complete()
            return

        # sync error: positive means A is ahead of B
        sync_error = travel_a - travel_b
        correction = self.sync_pid.calculate(sync_error)

        cmd_a = self.motor_a._calculate_cmd()
        cmd_b = self.motor_b._calculate_cmd()

        if not done_a:
            safe_a = max(0.0, min(self.motor_a.velocity, cmd_a + correction))
            self.motor_a.set_speed(safe_a * dir_a)
        else:
            self.motor_a.set_speed(0)

        if not done_b:
            safe_b = max(0.0, min(self.motor_b.velocity, cmd_b - correction))
            self.motor_b.set_speed(safe_b * dir_b)
        else:
            self.motor_b.set_speed(0)

    def move_by_counts(self, counts, is_blocking=True, on_complete=None):
        self._start_move(counts, counts,is_blocking, on_complete)
            
    def move_revolutions(self, distance_rev, is_blocking=True, on_complete=None):
        target_count = int(distance_rev * COUNTS_PER_REV)
        self.move_by_counts(target_count, is_blocking, on_complete)

    def move_degrees(self, distance_degree, is_blocking=True, on_complete=None):
        target_revolutions = distance_degree / 360
        self.move_revolutions(target_revolutions, is_blocking, on_complete)

    def move_cm(self, centimeters, is_blocking=True, on_complete=None):
        revolutions = centimeters / WHEEL_CIRCUMFERENCE
        self.move_revolutions(revolutions, is_blocking, on_complete)


    def turn_by_counts(self, counts, is_blocking=True, on_complete=None):
        # positive = clockwise: A forward, B backward
        self._start_move(counts, -counts, is_blocking, on_complete)

    def turn_by_degrees(self, degrees, is_blocking=True, on_complete=None):
        degrees = degrees * TURN_COEFFICIENT
        self.turn_by_counts(degrees, is_blocking, on_complete)

    def stop(self):
        self.motor_a.stop()
        self.motor_b.stop()

    def hold(self):
        self._sync_timer.deinit()
        self.motor_a.hold()
        self.motor_b.hold()

    def release(self):
        self.motor_a.release()
        self.motor_b.release()