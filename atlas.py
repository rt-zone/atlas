import time
import math
import _thread
from machine import Pin, PWM, I2C, time_pulse_us
from encoder import Encoder 
from pimoroni import PID 
import oled

class Atlas:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(Atlas, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self,
        # motors
        left_in1_pin=6, left_in2_pin=7, right_in1_pin=5, right_in2_pin=4,
        # ultrasonic
        echo_pin=13, trigger_pin=12,
        # oled i2c
        i2c_id=1, scl_pin=15, sda_pin=14, i2c_freq=400000,
        # encoders
        left_a=3, left_b=2, right_a=0, right_b=1,
        # line
        left_line=11, middle_line=10, right_line=9,
        # buzzer
        buzzer_pin=18):
        
        if self._initialized:
            return
        self._initialized = True

        # --- motors (PWMs) ---
        self.right_in1 = PWM(Pin(right_in1_pin))
        self.right_in2 = PWM(Pin(right_in2_pin))
        self.left_in1 = PWM(Pin(left_in1_pin))
        self.left_in2 = PWM(Pin(left_in2_pin))

        for m in (self.left_in1, self.left_in2, self.right_in1, self.right_in2):
            m.freq(1000)

        # --- ultrasonic ---
        self.echo = Pin(echo_pin, Pin.IN)
        self.trigger = Pin(trigger_pin, Pin.OUT)

        # --- OLED ---
        self.i2c = I2C(i2c_id, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=i2c_freq)
        self.oled = oled.OLED(self.i2c)
        self.vssa = 32

        # --- encoders ---
        self.left_enc  = Encoder(0, 0, (left_a, left_b))
        self.right_enc = Encoder(0, 1, (right_a, right_b))

        # --- line ---
        sensors = [Pin(left_line, Pin.IN), Pin(middle_line, Pin.IN), Pin(right_line, Pin.IN)]
        self.values = [1, 1, 1]
        sensor_index = {s: i for i, s in enumerate(sensors)}

        def handle_sensor(pin):
            self.values[sensor_index[pin]] = pin.value()

        for s in sensors:
            s.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=handle_sensor)

        # --- buzzer ---
        self.buzzer = PWM(Pin(buzzer_pin)) 
        self.buzzer.freq(560)
        self._volume = 50
        self.buzzer.duty_u16(0)

        self.NOTE = {
            "B3": 247,
            "C4": 262,
            "CS4": 277,
            "D4": 294,
            "DS4": 311,
            "E4": 330,
            "FS4": 370,
            "GS4": 415,
            "B4": 494,
            "G4": 392, # fail
            "A5": 880, # success
            "B5": 988,
            "C5": 523,
            "D5": 587,
            "E5": 659,
        }

        self.success_melody = [
            self.NOTE["A5"], self.NOTE["B5"], self.NOTE["C5"], self.NOTE["B5"], self.NOTE["C5"], 
            self.NOTE["D5"], self.NOTE["C5"], self.NOTE["D5"], self.NOTE["E5"], self.NOTE["D5"], 
            self.NOTE["E5"], self.NOTE["E5"]
        ]

        self.fail_melody = [
            self.NOTE["G4"], self.NOTE["C4"], self.NOTE["C4"]
        ]

        # --- PID setup (copy your tuned values) ---
        self.dt = 0.02
        
        # pid for turn
        self.left_pos_pid  = PID(0.002, 0.0, 0, self.dt)
        self.right_pos_pid = PID(0.002, 0.0, 0, self.dt)
        self.left_vel_pid  = PID(18, 0.0, 0.001, self.dt)
        self.right_vel_pid = PID(18, 0.0, 0.001, self.dt)
        self.align_pid = PID(0.0005, 0.0, 0, self.dt)

        # alternate pid for movement 
        self.left_pos_pid_f  = PID(0.002, 0.0, 0, self.dt)
        self.right_pos_pid_f = PID(0.002, 0.0, 0, self.dt)
        self.left_vel_pid_f  = PID(36, 0.0, 0.01, self.dt)
        self.right_vel_pid_f = PID(36, 0.0, 0.01, self.dt)
        self.align_pid_f = PID(0.00072, 0.0, 0, self.dt)

        # movement parameters
        self.WHEEL_DIAMETER_CM = 4.5
        self.wheel_circ = math.pi * self.WHEEL_DIAMETER_CM
        self.MAGIC_TURN_CONVERTER = 1.79    # maybe can be calculated

        # move speed (0..1)
        self.MoveSpeed = 1.0
        self.LeftSpeed = 1.0
        self.RightSpeed = 1.0

        # internal control flags
        self._stop = False                  # stops a motor function
        self._moving = False                # motor functions' state
        self._move_thread_running = False   # motor thread's state

    # ---------------------------
    # peripheries
    # ---------------------------
    # ultrasonic
    def getUltrasonicCm(self):
        self.trigger.value(0)
        time.sleep_us(2)
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        duration = time_pulse_us(self.echo, 1)
        distance = (duration * 0.0343) / 2
        return distance

    # oled
    def _print_line(self, msg):
        line_height = 8
        self.vssa -= line_height
        if self.vssa < 0:
            self.vssa = 0
            return
        # self.vscsad(self.vssa)
        y = self.oled.height - line_height - self.vssa
        self.oled.text(msg, 0, y)
        self.oled.show()

    def displayPrint(self, *args):
        max_chars = self.oled.width // 8
        msg = ("".join(str(a) for a in args))
        
        line = ""
        for char in msg:
            if char == '\n' or len(line) >= max_chars:
                self._print_line(line)
                line = "" if char == '\n' else char
            else:
                line += char
            
        if line:
            self._print_line(line)

    def displayClear(self):
        self.vssa = 32
        self.oled.fill(0)
        self.oled.show()

    # encoder
    def getEncoderCount(self, motor):
        if motor == 'A':
            return self.left_enc.capture().count
        else:
            return self.right_enc.capture().count
    
    def getEncoderDegrees(self, motor):
        if motor == 'A':
            return self.left_enc.capture().degrees
        else:
            return self.right_enc.capture().degrees
        
    def getEncoderRotations(self, motor):
        if motor == 'A':
            return self.left_enc.capture().revolutions
        else:
            return self.right_enc.capture().revolutions

    def getSpeedDegrees(self, motor):
        if motor == 'A':
            return self.left_enc.capture().degrees_per_second
        else:
            return self.right_enc.capture().degrees_per_second
        
    def getSpeedRotations(self, motor):
        if motor == 'A':
            return self.left_enc.capture().revolutions_per_second
        else:
            return self.right_enc.capture().revolutions_per_second

    # line
    def getLightLeft(self):
        return self.values[0]
    
    def getLightMiddle(self):
        return self.values[1]
    
    def getLightRight(self):
        return self.values[2]

    # buzzer
    def buzzerSetVolume(self, v):
        v = max(0.0, min(1.0, v / 100))
        self._volume = v        
        duty = v ** math.e
        self.buzzer.duty_u16(int(65535 * (duty / 2)))

    def buzzerStop(self):
        self.buzzer.duty_u16(0)

    def buzzerPlay(self, freq, volume, duration):
        self.buzzer.freq(int(freq))
        self.buzzerSetVolume(volume)
        time.sleep(duration)
        self.buzzerStop()

    def buzzerPlayBeep(self):
        self.buzzerPlay(2000, self._volume, 0.1)

    def buzzerPlayBoop(self):
        self.buzzerPlay(800, self._volume, 0.2)

    def buzzerPlaySuccess(self, volume=50):
        for i in range(2):
            for i in range(len(self.success_melody)):
                note_freq = self.success_melody[i]
                self.buzzerPlay(note_freq, volume, 0.07)

            self.buzzerStop()
            time.sleep(0.035)

    def buzzerPlayFail(self, volume=30):
        for i in range(len(self.fail_melody)):
            note_freq = self.fail_melody[i]
            self.buzzerPlay(note_freq, volume, 0.25)
        self.buzzerStop()

    # ---------------------------
    # motor helpers
    # ---------------------------
    def set_motor(self, pwm1, pwm2, speed):
        # speed expected -1..1, but original code used 0..1 only for forward
        if speed >= 0:
            pwm1.duty_u16(int(max(0, speed) * 65535))
            pwm2.duty_u16(0)
        else:
            pwm1.duty_u16(0)
            pwm2.duty_u16(int(max(0, -speed) * 65535))

    def move(self, left_speed, right_speed):
        # left_speed/right_speed between -1 and 1
        self.set_motor(self.left_in1, self.left_in2, left_speed)
        self.set_motor(self.right_in1, self.right_in2, right_speed)

    def stopMoveFunction(self):
        self._stop = True

    def stopMove(self):
        self.stopMoveFunction()
        self.move(0, 0)

    # ---------------------------
    # tune / speed
    # ---------------------------
    def setMoveSpeed(self, percent):
        speed = max(0, min(100, percent)) / 100.0
        self.MoveSpeed = speed
        self.LeftSpeed = speed
        self.RightSpeed = speed

    # Movement w/o PID
    def setSpeed(self, letter, percent):
        speed = max(0, min(100, percent)) / 100.0
        if letter == 'A':
            self.LeftSpeed = speed
        else: 
            self.RightSpeed = speed 

        self.stopMoveFunction()
        self.move(self.LeftSpeed, self.RightSpeed)
    
    def moveWithSpeeds(self, left, right):
        left_speed = max(0, min(100, left)) / 100.0
        right_speed = max(0, min(100, right)) / 100.0
        self.LeftSpeed = left_speed
        self.RightSpeed = right_speed
        
        self.stopMoveFunction()
        self.move(self.LeftSpeed, self.RightSpeed)

    # ---------------------------
    # single motor movement
    # ---------------------------
    def moveMotorDegrees(self, letter, distance_deg):
        direction = 1 if distance_deg >= 0 else -1
        distance_deg = abs(distance_deg)

        if letter == 'A':
            start_left = self.left_enc.capture().degrees
            self.set_motor(self.left_in1, self.left_in2, self.LeftSpeed * direction)

            try: 
                while True:
                    left_cap  = self.left_enc.capture().degrees
                    left_travel = abs(left_cap - start_left)
                    if (left_travel >= distance_deg):
                        break
            finally:
                self.set_motor(self.left_in1, self.left_in2, 0)

        else: 
            start_right = self.right_enc.capture().degrees
            self.set_motor(self.right_in1, self.right_in2, self.RightSpeed * direction)
            
            try: 
                while True:
                    right_cap = self.right_enc.capture().degrees
                    right_travel = abs(right_cap - start_right)
                    if (right_travel >= distance_deg):
                        break
            finally:
                self.set_motor(self.left_in1, self.left_in2, 0)

    def moveMotorRotations(self, letter, distance_rev):
        distance_deg = distance_rev * 360
        self.moveMotorDegrees(letter, distance_deg)

    def moveMotorCm(self, letter, distance_cm):
        distance_rev = distance_cm / self.wheel_circ
        distance_deg = distance_rev * 360
        self.moveMotorDegrees(letter, distance_deg)

    def moveMotorSeconds(self, letter, seconds): # negative seconds for reverse direction lol
        direction = 1 if seconds >= 0 else -1
        seconds = abs(seconds)

        if letter == 'A':
            self.set_motor(self.left_in1, self.left_in2, self.LeftSpeed * direction)
            time.sleep(seconds)
            self.set_motor(self.left_in1, self.left_in2, 0)
            
        else: 
            self.set_motor(self.right_in1, self.right_in2, self.RightSpeed * direction)
            time.sleep(seconds)
            self.set_motor(self.left_in1, self.left_in2, 0)

    def stopMotor(self, letter):
        if letter == 'A':
            self.set_motor(self.left_in1, self.left_in2, 0)
        else:
            self.set_motor(self.right_in1, self.right_in2, 0)
    
    # ---------------------------
    # helper used by both threaded and direct calls
    # ---------------------------
    def _find_k(self, travel, degrees_need):
        mds = 500
        mdt = 500
        k = 1
        remaining = degrees_need - travel
        if degrees_need < (mds + mdt):
            ds = degrees_need / 3
            dt = ds
            if travel <= ds:
                k = travel / ds
        else:
            if travel <= mds:
                k = travel / mds
        return max(0.15, min(1, k))

    # ---------------------------
    # turning (will also honor stop flag)
    # ---------------------------
    def turn(self, turn_degrees):
        a = 1
        if turn_degrees < 0:
            a = -1
        turn_degrees = abs(turn_degrees)

        distance_deg = turn_degrees * self.MAGIC_TURN_CONVERTER

        start_left = self.left_enc.capture().degrees
        start_right = self.right_enc.capture().degrees

        self.left_pos_pid.setpoint = distance_deg
        self.right_pos_pid.setpoint = distance_deg

        last_left_cmd = 0.15 * a
        last_right_cmd = 0.15 * (-a)

        self._stop = False
        self._moving = True
        try:
            while True:
                if self._stop:
                    self._moving = False
                    self._stop = False
                    return

                left_cap  = self.left_enc.capture()
                right_cap = self.right_enc.capture()

                left_travel = (left_cap.degrees - start_left) * a
                right_travel = (right_cap.degrees - start_right) * (-a)
                travel = (left_travel + right_travel) / 2

                if (travel >= distance_deg):
                    break

                left_speed = (left_cap.revolutions_per_second * a) / 4
                right_speed = (right_cap.revolutions_per_second * (-a)) / 4

                left_vel = self.left_pos_pid.calculate(left_travel)
                right_vel = self.right_pos_pid.calculate(right_travel)

                k = self._find_k(travel, distance_deg)

                self.left_vel_pid.setpoint = max(min(left_vel, k), -k)
                self.right_vel_pid.setpoint = max(min(right_vel, k), -k)

                left_accel = self.left_vel_pid.calculate(left_speed)
                right_accel = self.right_vel_pid.calculate(right_speed)

                last_left_cmd += left_accel * self.dt
                last_right_cmd += right_accel * self.dt

                speed_error = left_travel - right_travel
                correction = self.align_pid.calculate(speed_error)

                last_left_cmd  += correction
                last_right_cmd -= correction

                last_left_cmd = max(min(last_left_cmd, 1), 0)
                last_right_cmd = max(min(last_right_cmd, 1), 0)

                final_left = last_left_cmd * a
                final_right = last_right_cmd * (-a)
                self.move(final_left, final_right)

                time.sleep(self.dt)
        finally:
            self.move(0, 0)
            self._moving = False
            self._stop = False

    def turnRightDegrees(self, degrees):
        self.turn(degrees)

    def turnLeftDegrees(self, degrees):
        self.turn(-degrees)

    # ---------------------------
    # move specific distance (respects stop flag)
    # ---------------------------
    def moveForwardCm(self, distance_cm):
        # distance_cm can be negative for backwards motion
        distance_rev = distance_cm / self.wheel_circ
        distance_deg = abs(distance_rev * 360)

        start_left = self.left_enc.capture().degrees
        start_right = self.right_enc.capture().degrees

        self.left_pos_pid_f.setpoint = distance_deg
        self.right_pos_pid_f.setpoint = distance_deg

        last_left_cmd = 0.0
        last_right_cmd = 0.0

        direction = 1 if distance_cm >= 0 else -1

        self._stop = False
        self._moving = True
        try:
            while True:
                if self._stop:
                    self._moving = False
                    self._stop = False
                    return

                left_cap  = self.left_enc.capture()
                right_cap = self.right_enc.capture()

                # use average of absolute degrees travelled
                left_travel = abs(left_cap.degrees - start_left)
                right_travel = abs(right_cap.degrees - start_right)
                traveled = (left_travel + right_travel) / 2

                if distance_deg != math.inf and (traveled >= distance_deg):
                    break

                left_speed = left_cap.revolutions_per_second / 4
                right_speed = right_cap.revolutions_per_second / 4

                left_vel = self.left_pos_pid_f.calculate(left_travel)
                right_vel = self.right_pos_pid_f.calculate(right_travel)

                k = self._find_k(traveled, distance_deg if distance_deg != math.inf else 1000)

                self.left_vel_pid_f.setpoint = max(min(left_vel, self.MoveSpeed), -self.MoveSpeed)
                self.right_vel_pid_f.setpoint = max(min(right_vel, self.MoveSpeed), -self.MoveSpeed)

                left_accel = self.left_vel_pid_f.calculate(left_speed)
                right_accel = self.right_vel_pid_f.calculate(right_speed)

                last_left_cmd += k * k * left_accel * self.dt
                last_right_cmd += k * k * right_accel * self.dt

                speed_error = (left_cap.degrees - start_left) - (right_cap.degrees - start_right)
                correction = self.align_pid_f.calculate(speed_error)

                last_left_cmd  += correction
                last_right_cmd -= correction

                last_left_cmd = max(min(last_left_cmd, self.MoveSpeed), 0)
                last_right_cmd = max(min(last_right_cmd, self.MoveSpeed), 0)

                # apply forward/backward direction
                self.move(last_left_cmd * direction, last_right_cmd * direction)

                time.sleep(self.dt)
        finally:
            self.move(0, 0)
            self._moving = False
            self._stop = False

    def moveBackwardCm(self, distance_cm):
        self.moveForwardCm(-distance_cm)

    def moveForwardRotations(self, distance_rev):
        distance_cm = distance_rev * self.wheel_circ
        self.moveForwardCm(distance_cm)

    def moveBackwardRotations(self, distance_rev):
        self.moveForwardRotations(-distance_rev)

    def moveForwardDegrees(self, distance_deg):
        distance_rev = distance_deg * 360
        distance_cm = distance_rev * self.wheel_circ
        self.moveForwardCm(distance_cm)

    def moveBackwardDegrees(self, distance_deg):
        self.moveForwardDegrees(distance_deg)

    # ---------------------------
    # non-blocking helpers: start movement in a thread when available
    # ---------------------------
    def _start_threaded(self, target, *args):
        if self._move_thread_running:
            print("[Atlas] a movement thread is already running")
            return

        def thread_target(arg_tuple):
            self._move_thread_running = True
            try:
                target(*arg_tuple)
            finally:
                self._move_thread_running = False

        _thread.start_new_thread(thread_target, (args,))

    # infinite movement methods that return immediately (if _thread is available)
    def moveForward(self):
        self.stopMoveFunction()       
        """Start moving forward indefinitely (non-blocking when _thread available)."""
        self._start_threaded(self.moveForwardCm, 100000)

    def moveBackward(self):
        self.stopMoveFunction()
        """Start moving backward indefinitely (non-blocking when _thread available)."""
        self._start_threaded(self.moveForwardCm, -100000)       
