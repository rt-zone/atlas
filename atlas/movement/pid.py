class PID:
    def __init__(self, kp, ki, kd, dt, i_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.dt = dt
        self.i_limit = i_limit

        self.i = 0
        self.prev_error = 0

    def update(self, target, measurement):
        error = target - measurement

        # P
        p = self.kp * error

        # I (with clamp)
        self.i += error * self.dt
        self.i = max(-self.i_limit, min(self.i, self.i_limit))

        # D
        d = (error - self.prev_error) / self.dt
        self.prev_error = error

        return p + self.ki * self.i + self.kd * d