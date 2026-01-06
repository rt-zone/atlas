# MPU-6050 Simple MicroPython Library with Tilt Event Support
# MIT License

import time

class MPU6050:
    def __init__(self, i2c, address=0x68):
        self.i2c = i2c
        self.address = address

        try:
            self.wake()
            time.sleep_ms(100)
        except Exception as e:
            raise RuntimeError("Failed to wake up GyroAxel. Check wiring and connection.") from e

        self._tilt_listeners = {
            "forward": [],
            "backward": [],
            "left": [],
            "right": []
        }
        self._tilt_threshold = 0.4

    def wake(self):
        self.i2c.writeto_mem(self.address, 0x6B, b'\x00')

    def sleep(self):
        self.i2c.writeto_mem(self.address, 0x6B, b'\x40')

    def read_temperature(self):
        data = self.i2c.readfrom_mem(self.address, 0x41, 2)
        raw = self._combine(data[0], data[1])
        return (raw / 340.0) + 36.53

    def read_accel(self):
        data = self.i2c.readfrom_mem(self.address, 0x3B, 6)
        x = self._combine(data[0], data[1]) / 16384.0
        y = self._combine(data[2], data[3]) / 16384.0
        z = self._combine(data[4], data[5]) / 16384.0
        return (x, y, z)

    def read_gyro(self):
        data = self.i2c.readfrom_mem(self.address, 0x43, 6)
        x = self._combine(data[0], data[1]) / 131.0
        y = self._combine(data[2], data[3]) / 131.0
        z = self._combine(data[4], data[5]) / 131.0
        return (x, y, z)

    def read(self):
        temp = self.read_temperature()
        ax, ay, az = self.read_accel()
        gx, gy, gz = self.read_gyro()
        return {'temp':temp, 'accel':(ax,ay,az), 'gyro':(gx,gy,gz)}

    def add_tilt_listener(self, direction, callback):
        if direction not in self._tilt_listeners:
            raise ValueError("Direction must be 'forward', 'backward', 'left', or 'right'")
        self._tilt_listeners[direction].append(callback)

    def check_tilt(self):
        x, y, _ = self.read_accel()
        if x > self._tilt_threshold:
            for fn in self._tilt_listeners["forward"]:
                fn()
        elif x < -self._tilt_threshold:
            for fn in self._tilt_listeners["backward"]:
                fn()
        if y > self._tilt_threshold:
            for fn in self._tilt_listeners["left"]:
                fn()
        elif y < -self._tilt_threshold:
            for fn in self._tilt_listeners["right"]:
                fn()

    def _combine(self, high, low):
        value = (high << 8) | low
        return value - 65536 if value >= 0x8000 else value
