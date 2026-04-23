# import time
# import math
# import _thread
# from machine import Pin, PWM
# from PID import PID 

# from .modules.OLED import OLED
# from .modules.Ultrasonic import Ultrasonic
# from .modules.LineSensor import LineSensor
# from .modules.Buzzer import Buzzer

# PWM_FREQ = 20_000

# def _speed2power(speed):
#     direction = 1 if speed > 0 else -1
#     return (abs(speed) ** 0.15 * direction)

# def _find_k(travel, degrees_need):
#         mds = 100
#         mdt = 100
#         k = 1
#         if degrees_need < (mds + mdt):
#             ds = degrees_need / 3
#             if travel <= ds:
#                 k = travel / ds
#         else:
#             if travel <= mds:
#                 k = travel / mds
#         return max(0.2, min(1, k))

# def _find_k_turn(travel, degrees_need):
#     mds = 50
#     k = 1
#     if travel <= mds:
#         k = (travel / mds) * 0.4
#     return max(0.1, min(0.4, k))

# class Atlas:
#     _instance = None

#     def __new__(cls, *args, **kwargs):
#         if cls._instance is None:
#             cls._instance = super(Atlas, cls).__new__(cls)
#             cls._instance._initialized = False
#         return cls._instance
    
#     def __init__(self):
#         if self._initialized:
#             return
#         self._initialized = True

#         # --- motors (PWMs) ---
#         self.right_in1 = PWM(Pin("MOTOR_A1"), freq=PWM_FREQ)
#         self.right_in2 = PWM(Pin("MOTOR_A2"), freq=PWM_FREQ)
#         self.left_in1 = PWM(Pin("MOTOR_B1"), freq=PWM_FREQ)
#         self.left_in2 = PWM(Pin("MOTOR_B2"), freq=PWM_FREQ)

        

#         # --- PID setup (copy your tuned values) ---
#         self.dt = 0.02
        
#         # pid for turn
#         self.left_pos_pid  = PID(0.005, 0.0, 0, self.dt)
#         self.right_pos_pid = PID(0.005, 0.0, 0, self.dt)
#         self.left_vel_pid  = PID(10, 0.0, 0.001, self.dt)
#         self.right_vel_pid = PID(10, 0.0, 0.001, self.dt)
#         self.align_pid = PID(0, 0.0, 0, self.dt)

#         # alternate pid for movement 
#         self.left_pos_pid_f  = PID(0.002, 0.0, 0, self.dt)
#         self.right_pos_pid_f = PID(0.002, 0.0, 0, self.dt)
#         self.left_vel_pid_f  = PID(36, 0.0, 0.0005, self.dt)
#         self.right_vel_pid_f = PID(36, 0.0, 0.0005, self.dt)
#         self.align_pid_f = PID(0.00072, 0.0, 0.0001, self.dt)

#         # movement parameters
#         self.WHEEL_DIAMETER_CM = 4.5
#         self.wheel_circ = math.pi * self.WHEEL_DIAMETER_CM
#         self.MAGIC_TURN_CONVERTER = 1.79    # maybe can be calculated

#         # move speed (0..1)
#         self.MoveSpeed = 1.0
#         self.LeftSpeed = 1.0
#         self.RightSpeed = 1.0

#         # internal control flags
#         self._stop = False                  # stops a motor function
#         self._moving = False                # motor functions' state
#         self._move_thread_running = False   # motor thread's state


#     # ---------------------------
#     # motor helpers
#     # ---------------------------
#     def _set_motor(self, pwm1, pwm2, speed):
#         # speed expected -1..1, but original code used 0..1 only for forward
#         if speed >= 0:
#             pwm1.duty_u16(int(max(0, speed) * 65535))
#             pwm2.duty_u16(0)
#         else:
#             pwm1.duty_u16(0)
#             pwm2.duty_u16(int(max(0, -speed) * 65535))
    
#     def _move(self, left_speed, right_speed):
#         # left_speed/right_speed between -1 and 1
#         left_power = _speed2power(left_speed)
#         right_power = _speed2power(right_speed)
        
#         self._set_motor(self.left_in1, self.left_in2, left_power)
#         self._set_motor(self.right_in1, self.right_in2, right_power)

    
#     def _stop_smooth(self):
#         left_speed  = encoders('A').capture().revolutions_per_second / 4
#         right_speed = encoders('B').capture().revolutions_per_second / 4


#         l = -1
#         r = -1
#         if left_speed <= 0:
#             l = 1
#         if right_speed <= 0:
#             r = 1

#         sleep_time = abs(left_speed + right_speed) / 40
#         self._move(l,r)
#         time.sleep(sleep_time)
#         self._move(0,0)
#         time.sleep(0.5)
#         self._moving = False
#         print("smooth stop")

#     def _stop_move_thread(self): # stop thread, don't stop moving
#         if self._move_thread_running:
#             self._stop = True
#         t = time.ticks_ms()
#         while self._move_thread_running:
#             time.sleep_ms(5)
#             if time.ticks_ms() - t >= 1000:
#                 print("[Atlas] couldn't stop a running thread")
#                 break
#         print("thread stopped")

#     def stop_move(self):
#         self._stop_move_thread()
#         self._stop_smooth()

#     # ---------------------------
#     # tune / speed
#     # ---------------------------
#     def set_move_speed(self, percent):
#         """
#             Speed in percents (0 - 100)
#         """
#         speed = max(0, min(100, percent)) / 100.0
#         self.MoveSpeed = speed
#         self.set_speed('A', speed)
#         self.set_speed('B', speed)
        
#     # Movement w/o PID
#     def set_speed(self, letter, percent):
#         speed = max(0, min(100, percent)) / 100.0
#         if letter == 'A':
#             self.LeftSpeed = speed
#         else: 
#             self.RightSpeed = speed 
    
#     def moveWithSpeeds(self, left, right):
#         left_speed = max(0, min(100, left)) / 100.0
#         right_speed = max(0, min(100, right)) / 100.0
#         self.LeftSpeed = left_speed
#         self.RightSpeed = right_speed
#         self._stop_move_thread()
#         self._move(self.LeftSpeed, self.RightSpeed)

#     # ---------------------------
#     # single motor movement
#     # ---------------------------



#     def moveMotorRotations(self, letter, distance_rev):
#         distance_deg = distance_rev * 360
#         self.moveMotorDegrees(letter, distance_deg)

#     def moveMotorCm(self, letter, distance_cm):
#         distance_rev = distance_cm / self.wheel_circ
#         distance_deg = distance_rev * 360
#         self.moveMotorDegrees(letter, distance_deg)

#     def moveMotorSeconds(self, letter, seconds): # negative seconds for reverse direction lol
#         direction = 1 if seconds >= 0 else -1
#         seconds = abs(seconds)

#         if letter == 'A':
#             self._set_motor(self.left_in1, self.left_in2, self.LeftSpeed * direction)
#             time.sleep(seconds)
#             self._set_motor(self.left_in1, self.left_in2, 0)
            
#         else: 
#             self._set_motor(self.right_in1, self.right_in2, self.RightSpeed * direction)
#             time.sleep(seconds)
#             self._set_motor(self.left_in1, self.left_in2, 0)

#     def stopMotor(self, letter):
#         if letter == 'A':
#             self._set_motor(self.left_in1, self.left_in2, 0)
#         else:
#             self._set_motor(self.right_in1, self.right_in2, 0)
    
#     # ---------------------------
#     # helper used by both threaded and direct calls
#     # ---------------------------
    

#     # ---------------------------
#     # turning (will also honor stop flag)
#     # ---------------------------
#     def turn(self, turn_degrees):
#         a = 1
#         if turn_degrees < 0:
#             a = -1
#         turn_degrees = abs(turn_degrees)

#         distance_deg = turn_degrees * self.MAGIC_TURN_CONVERTER

#         if turn_degrees >= 360:
#             degrees_to_stop = 62 # 360+
#         elif turn_degrees >= 270:
#             degrees_to_stop = 70 # 270
#         elif turn_degrees >= 180:
#             degrees_to_stop = 70 # 180
#         elif turn_degrees >= 90: 
#             degrees_to_stop = 74 # 90
#         elif turn_degrees >= 80:
#             degrees_to_stop = 70 # 70
#         elif turn_degrees >= 70:
#             degrees_to_stop = 60 # 70
#         elif turn_degrees >= 60:
#             degrees_to_stop = 50 # 60
#         elif turn_degrees >= 45:
#             degrees_to_stop = 25 # 45
#         else:
#             degrees_to_stop = turn_degrees / 2 


#         start_left = encoders('A').capture().degrees
#         start_right = encoders('B').capture().degrees

#         self.left_pos_pid.setpoint = distance_deg
#         self.right_pos_pid.setpoint = distance_deg

#         last_left_cmd = 0.0
#         last_right_cmd = 0.0

#         self._moving = True
#         try:
#             while True:
#                 if self._stop:
#                     self._stop = False
#                     return

#                 left_cap  = encoders('A').capture()
#                 right_cap = encoders('B').capture()

#                 left_travel = (left_cap.degrees - start_left) * a
#                 right_travel = (right_cap.degrees - start_right) * (-a)
#                 travel = (left_travel + right_travel) / 2

#                 left_speed = (left_cap.revolutions_per_second * a) / 4
#                 right_speed = (right_cap.revolutions_per_second * (-a)) / 4

#                 if (travel >= distance_deg - degrees_to_stop):
#                     break

#                 left_vel = self.left_pos_pid.calculate(left_travel)
#                 right_vel = self.right_pos_pid.calculate(right_travel)

#                 k = _find_k_turn(travel, distance_deg)

#                 self.left_vel_pid.setpoint = max(min(left_vel, k), -k)
#                 self.right_vel_pid.setpoint = max(min(right_vel, k), -k)

#                 left_accel = self.left_vel_pid.calculate(left_speed)
#                 right_accel = self.right_vel_pid.calculate(right_speed)

#                 last_left_cmd += left_accel * self.dt
#                 last_right_cmd += right_accel * self.dt

#                 speed_error = left_travel - right_travel
#                 correction = self.align_pid.calculate(speed_error)

#                 last_left_cmd  += correction
#                 last_right_cmd -= correction

#                 last_left_cmd = max(min(last_left_cmd, 1), 0)
#                 last_right_cmd = max(min(last_right_cmd, 1), 0)

#                 final_left = last_left_cmd * a
#                 final_right = last_right_cmd * (-a)
#                 self._move(final_left, final_right)

#                 time.sleep(self.dt)
#         finally:
#             self._stop_smooth()

#     def turnRightDegrees(self, degrees):
#         self.turn(degrees)

#     def turnLeftDegrees(self, degrees):
#         self.turn(-degrees)

#     # ---------------------------
#     # move specific distance (respects stop flag)
#     # ---------------------------
#     def moveForwardCm(self, distance_cm):
#         # distance_cm can be negative for backwards motion
#         direction = 1 if distance_cm >= 0 else -1
#         distance_cm = abs(distance_cm)

#         distance_rev = distance_cm / self.wheel_circ
#         distance_deg = distance_rev * 360 + 90

#         start_left = encoders('A').capture().degrees
#         start_right = encoders('B').capture().degrees

#         self.left_pos_pid_f.setpoint = distance_deg
#         self.right_pos_pid_f.setpoint = distance_deg

#         last_left_cmd = 0.0
#         last_right_cmd = 0.0

#         self._moving = True
#         try:
#             while True:
#                 if self._stop:
#                     self._stop = False
#                     return

#                 left_cap  = encoders('A').capture()
#                 right_cap = encoders('B').capture()

#                 # use average of absolute degrees travelled
#                 left_travel = abs(left_cap.degrees - start_left)
#                 right_travel = abs(right_cap.degrees - start_right)
#                 traveled = (left_travel + right_travel) / 2

#                 left_speed = abs(left_cap.revolutions_per_second) / 4
#                 right_speed = abs(right_cap.revolutions_per_second) / 4

#                 if traveled >= distance_deg - 90:
#                     self._stop_smooth()
#                     break

#                 left_vel = self.left_pos_pid_f.calculate(left_travel)
#                 right_vel = self.right_pos_pid_f.calculate(right_travel)

#                 k = _find_k(traveled, distance_deg)

#                 self.left_vel_pid_f.setpoint = max(min(left_vel, self.MoveSpeed), -self.MoveSpeed)
#                 self.right_vel_pid_f.setpoint = max(min(right_vel, self.MoveSpeed), -self.MoveSpeed)

#                 left_accel = self.left_vel_pid_f.calculate(left_speed)
#                 right_accel = self.right_vel_pid_f.calculate(right_speed)

#                 last_left_cmd += k * k * left_accel * self.dt
#                 last_right_cmd += k * k * right_accel * self.dt

#                 speed_error = abs(left_cap.degrees - start_left) - abs(right_cap.degrees - start_right)
#                 correction = self.align_pid_f.calculate(speed_error)

#                 last_left_cmd  += correction
#                 last_right_cmd -= correction

#                 last_left_cmd = max(min(last_left_cmd, self.MoveSpeed), 0)
#                 last_right_cmd = max(min(last_right_cmd, self.MoveSpeed), 0)

#                 # apply forward/backward direction
#                 self._move(last_left_cmd * direction, last_right_cmd * direction)

#                 time.sleep(self.dt)
#         finally:
#             pass

#     def moveBackwardCm(self, distance_cm):
#         self.moveForwardCm(-distance_cm)

#     def moveForwardRotations(self, distance_rev):
#         distance_cm = distance_rev * self.wheel_circ
#         self.moveForwardCm(distance_cm)

#     def moveBackwardRotations(self, distance_rev):
#         self.moveForwardRotations(-distance_rev)

#     def moveForwardDegrees(self, distance_deg):
#         distance_rev = distance_deg * 360
#         distance_cm = distance_rev * self.wheel_circ
#         self.moveForwardCm(distance_cm)

#     def moveBackwardDegrees(self, distance_deg):
#         self.moveForwardDegrees(distance_deg)

#     # ---------------------------
#     # non-blocking helpers: start movement in a thread when available
#     # ---------------------------
#     def _start_threaded(self, target, *args):
#         if self._move_thread_running:
#             print("[Atlas] a movement thread is already running")
#             return

#         def thread_target(arg_tuple):
#             self._move_thread_running = True
#             try:
#                 target(*arg_tuple)
#             finally:
#                 self._move_thread_running = False

#         _thread.start_new_thread(thread_target, (args,))

#     # infinite movement methods that return immediately (if _thread is available)
#     def moveForward(self):
#         self._stop_move_thread()       
#         """Start moving forward indefinitely (non-blocking when _thread available)."""
#         self._start_threaded(self.moveForwardCm, 100000)

#     def moveBackward(self):
#         self._stop_move_thread()
#         """Start moving backward indefinitely (non-blocking when _thread available)."""
#         self._start_threaded(self.moveForwardCm, -100000)       
