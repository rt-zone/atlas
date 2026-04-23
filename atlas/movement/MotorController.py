from machine import Timer
from .Motor import Motor, COUNTS_PER_REV, WHEEL_CIRCUMFERENCE
from .pid import PID

SYNC_PID_PERIOD = 0.01

def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))

class MotorController:
    def __init__(self, velocity=1):
        self.motor_a = Motor('A', velocity)
        self.motor_b = Motor('B', velocity)

        self.sync_pid = PID(0.001, 0.0, 0.0, SYNC_PID_PERIOD)
        self.sync_pid.setpoint = 0

        self._sync_timer = Timer()

    def _start_move(self, count_a, count_b, on_complete):
        self._sync_timer.deinit()  # always stop previous timer before starting a new move

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

    def _sync_tick(self, dir_a, dir_b, on_complete, timer):
        travel_a = self.motor_a._get_travel()
        travel_b = self.motor_b._get_travel()

        done_a = travel_a >= abs(self.motor_a.distance_count)
        done_b = travel_b >= abs(self.motor_b.distance_count)

        if done_a and done_b:
            self.hold()
            if on_complete:
                on_complete()
            return

        sync_error = travel_a - travel_b
        correction = self.sync_pid.calculate(sync_error)

        cmd_a = self.motor_a._calculate_cmd()
        cmd_b = self.motor_b._calculate_cmd()

        if not done_a:
            # Clamp before applying direction so correction never causes accidental reversal
            safe_a = clamp(cmd_a + correction, 0.0, self.motor_a.velocity)
            self.motor_a.set_speed(safe_a * dir_a)
        else:
            self.motor_a.set_speed(0)

        if not done_b:
            safe_b = clamp(cmd_b - correction, 0.0, self.motor_b.velocity)
            self.motor_b.set_speed(safe_b * dir_b)
        else:
            self.motor_b.set_speed(0)

    def move(self, counts, on_complete=None):
        self._start_move(counts, counts, on_complete)

    def move_cm(self, cm, on_complete=None):
        counts = int((cm / WHEEL_CIRCUMFERENCE) * 357.75)
        self.move(counts, on_complete)

    def turn(self, counts, on_complete=None):
        # positive = clockwise: A forward, B backward
        self._start_move(counts, -counts, on_complete)

    def stop(self):
        self._sync_timer.deinit()
        self.motor_a.stop()
        self.motor_b.stop()

    def hold(self):
        self._sync_timer.deinit()
        # Hold at intended target, not wherever the motor happened to stop
        self.motor_a.hold(self.motor_a.start_count + self.motor_a.distance_count)
        self.motor_b.hold(self.motor_b.start_count + self.motor_b.distance_count)

    def release(self):
        self.motor_a.release()
        self.motor_b.release()

