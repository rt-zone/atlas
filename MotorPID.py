from machine import Timer
from PID import PID  # your PID library


# ─────────────────────────────────────────────
#  Anti-windup wrapper around the provided PID
# ─────────────────────────────────────────────
class PIDWithWindup(PID):
    """
    Extends PID with integral clamping (anti-windup).

    Without this, when the motor output is saturated at 0 or 1,
    _error_sum keeps growing even though it has no effect — the
    motor can't go any faster/slower. When the setpoint finally
    changes the integrator 'unwinds' slowly causing delayed and
    overshooting behaviour.

    max_integral: maximum absolute value of the integral accumulator.
    A value of 1.0 / ki is a sensible starting point (limits I-term
    contribution to ±1.0 in the output).
    """
    def __init__(self, kp, ki, kd, sample_rate, max_integral=1.0):
        super().__init__(kp, ki, kd, sample_rate)
        self.max_integral = max_integral

    def calculate(self, value, value_change=None):
        output = super().calculate(value, value_change)
        # Clamp the accumulator AFTER this step so it stays bounded
        self._error_sum = max(-self.max_integral,
                              min(self.max_integral, self._error_sum))
        return output

    def reset(self):
        """Clear integrator and derivative memory (useful on setpoint jumps)."""
        self._error_sum = 0
        self._last_value = 0


# ─────────────────────────────────────────────
#  Motor speed controller
# ─────────────────────────────────────────────
class MotorSpeedController:
    """
    Closed-loop speed controller for a single motor.

    Args:
        motor:       motor object with a .move(speed: float) method (0..1)
        encoder:     encoder object with a .get_speed() method (0..1)
        kp, ki, kd:  PID gains — start with ki=kd=0 and tune kp first
        sample_rate: control loop period in seconds (e.g. 0.02 = 50 Hz)
        max_integral: anti-windup clamp on the integral accumulator
    """

    SAMPLE_RATE = 0.02          # 50 Hz default — good for DC motors
    MAX_INTEGRAL = 1.0          # integral clamping limit

    def __init__(self, motor, encoder,
                 kp=0.5, ki=0.1, kd=0.01,
                 sample_rate=SAMPLE_RATE,
                 max_integral=MAX_INTEGRAL):

        self.motor = motor
        self.encoder = encoder
        self.sample_rate = sample_rate

        self.pid = PIDWithWindup(kp, ki, kd, sample_rate, max_integral)
        self.pid.setpoint = 0.0

        self._output = 0.0      # last computed output (useful for debugging)
        self._timer = None

    # ── public API ────────────────────────────

    def set_speed(self, target: float):
        """
        Set the desired speed (0.0 – 1.0).
        Clamps the value so the PID never chases an unreachable setpoint.
        """
        self.pid.setpoint = max(0.0, min(1.0, target))

    def start(self):
        """Start the control loop on a hardware timer."""
        self.pid.reset()
        period_ms = int(self.sample_rate * 1000)
        self._timer = Timer()
        self._timer.init(
            period=period_ms,
            mode=Timer.PERIODIC,
            callback=self._update   # runs every `sample_rate` seconds
        )

    def stop(self):
        """Stop the control loop and cut motor power."""
        if self._timer:
            self._timer.deinit()
            self._timer = None
        self.motor.move(0)
        self._output = 0.0

    @property
    def speed(self) -> float:
        """Current measured speed (0.0 – 1.0)."""
        return self.encoder.get_speed()

    @property
    def output(self) -> float:
        """Last PWM output sent to the motor (0.0 – 1.0)."""
        return self._output

    # ── internal ──────────────────────────────

    def _update(self, timer):
        """
        Timer callback — runs at every sample_rate tick.

        1. Read the encoder speed (process variable)
        2. Feed it to the PID → get a correction output
        3. Clamp the output to [0, 1] (motor can't go negative or >100%)
        4. Send it to the motor
        """
        measured = self.encoder.get_speed()

        raw_output = self.pid.calculate(measured)

        # Hard clamp: motor only accepts 0..1
        self._output = max(0.0, min(1.0, raw_output))

        self.motor.move(self._output)


# ─────────────────────────────────────────────
#  Example usage
# ─────────────────────────────────────────────
if __name__ == "__main__":
    import utime

    # Replace these with your actual motor / encoder imports and setup
    # from motor import Motor
    # from encoder import Encoder
    # motor   = Motor(...)
    # encoder = Encoder(...)

    # ── Tuning guide ──────────────────────────
    #
    #  Step 1: set ki=0, kd=0, increase kp until the motor
    #          reaches the setpoint reasonably fast but without
    #          too much oscillation.
    #
    #  Step 2: increase ki slowly to eliminate the remaining
    #          steady-state error (motor stuck just below target).
    #          Watch out for oscillation — lower kp slightly if needed.
    #
    #  Step 3: add a small kd to reduce overshoot if the motor
    #          surges past the setpoint on sudden changes.
    #
    #  Typical starting point for small DC motors:
    #    kp=0.5, ki=0.1, kd=0.01
    # ─────────────────────────────────────────

    ctrl = MotorSpeedController(
        motor=motor,
        encoder=encoder,
        kp=0.5,
        ki=0.1,
        kd=0.01,
        sample_rate=0.02,    # 50 Hz
        max_integral=1.0,
    )

    ctrl.set_speed(0.6)      # target: 60% of max speed
    ctrl.start()

    try:
        while True:
            # Optional: print live telemetry for tuning
            print(f"SP={ctrl.pid.setpoint:.2f}  "
                  f"PV={ctrl.speed:.2f}  "
                  f"OUT={ctrl.output:.2f}")
            utime.sleep(0.1)

    except KeyboardInterrupt:
        ctrl.stop()
        print("Stopped.")