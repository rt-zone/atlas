import gc
import time
import math
from machine import Pin, I2C, ADC
from pimoroni import PID
from motor import Motor
from encoder import Encoder
from ssd1306 import SSD1306_I2C
gc.enable()

class ArtisanBot():
    MOTOR_A_PINS = (4, 5)
    MOTOR_B_PINS = (6, 7)
    ENCODER_A_PINS = (0, 1)
    ENCODER_B_PINS = (2, 3)
    
    USER_SW_PIN = 8
    SPEED_SCALE = 5.4
    # PID values
    POS_KP = 0.25
    POS_KI = 0.0
    POS_KD = 0.01
    
    WHEEL_DIA = 4.3
    
    ONETURN = 84/43 # degrees of robot turning per degree of motor rotation
    
    def __init__(self, motor_gear_ratio=112, init_motors=True):
        # Free up hardware resources for encoders
        gc.collect()
        
        cpr = 12 * motor_gear_ratio
        self.motors = [Motor(self.MOTOR_A_PINS, direction=1,speed_scale=self.SPEED_SCALE), Motor(self.MOTOR_B_PINS, direction=0, speed_scale=self.SPEED_SCALE)]
        
        # Set the encoders to use PIO 0 and State Machines 0 and 1
        self.encoders = [Encoder(0, 0, self.ENCODER_A_PINS, counts_per_rev=cpr, count_microsteps=True, direction=0),
                         Encoder(0, 1, self.ENCODER_B_PINS, counts_per_rev=cpr, count_microsteps=True, direction=1)]
        # Set up the user switch
        self.__switch = Pin(self.USER_SW_PIN, Pin.IN, Pin.PULL_DOWN)
        # Set up the OLED display
        i2c = I2C(1, sda=Pin(14), scl=Pin(15))
        self.oled = SSD1306_I2C(128, 32, i2c)
        self.pid_kp = self.POS_KP
        self.pid_ki = self.POS_KI
        self.pid_kd = self.POS_KD
        self.curA = ADC(26)
        self.curB = ADC(27)
        
    def text(self, text, posx, posy):
        self.oled.text(text,posx, posy)
        
    def show(self):
        self.oled.show()
        
    def rotate(self, r):
        self.oled.rotate(r)
        
    def rect(self,x,y,w,h,v):
        self.oled.rect(x,y,w,h,v)
    
    def fill_rect(self,x,y,w,h,v):
        self.oled.fill_rect(x,y,w,h,v)
    
    def hline(self, x,y,l,v):
        self.oled.hline(x,y,l,v)
    
    def vline(self, x,y,l,v):
        self.oled.vline(x,y,l,v)
    
    def switch_pressed(self):
        return self.__switch.value()
    
    def move(self, setA, setB, speed=1):
        START_POS = [self.encoders[0].capture().degrees, self.encoders[1].capture().degrees]
        SETPOINT = max(abs(setA), abs(setB))
        
        UPDATES = 100
        UPDATE_RATE = 1 / UPDATES
        TIME_FOR_EACH_MOVE = 1*SETPOINT/360
        UPDATES_PER_MOVE = TIME_FOR_EACH_MOVE * UPDATES
        UPDATE_RATE = 1 / UPDATES
        
        # Create PID object for position control
        pos_pid = (PID(self.POS_KP, self.POS_KI, self.POS_KD, UPDATE_RATE),
                   PID(self.POS_KP, self.POS_KI, self.POS_KD, UPDATE_RATE))
        # Enable the motor to get started
        self.motors[0].enable()
        self.motors[1].enable()
        
        # Set starting and ending values
        start_value = START_POS
        end_value   = [START_POS[0]+setA, START_POS[1]+setB]
        
        for u in range(UPDATES_PER_MOVE):
            # Capture the state of the encoder
            capture = [self.encoders[0].capture(), self.encoders[1].capture()]
            # Calculate how far along this movement to be
            percent_along = u / UPDATES_PER_MOVE
            for i in range(2):
                pos_pid[i].setpoint = (((-math.cos(percent_along * math.pi) + 1.0) / 2.0) *
                                       (end_value[i] - start_value[i])) + start_value[i]
            # Set the new motor driving speed
            self.motors[0].speed(pos_pid[0].calculate(capture[0].degrees, capture[0].degrees_per_second))
            self.motors[1].speed(pos_pid[1].calculate(capture[1].degrees, capture[1].degrees_per_second))
            
            #print("U=",u ," PosA =", capture[0].degrees, "SetA = ", end_value[0], end=", ")
            #print("PosB =", capture[1].degrees, "SetB = ", end_value[1])
            
            time.sleep(UPDATE_RATE)
            
        self.motors[0].brake()
        self.motors[1].brake()
        
    def turnRight(self, deg):
        self.move(-deg*self.ONETURN,deg*self.ONETURN)
        return 1
    
    def turnLeft(self, deg):
        self.move(deg*self.ONETURN,-deg*self.ONETURN)
        return 1

    def moveFwd(self, cm):
        self.move(cm*360/(self.WHEEL_DIA*math.pi),cm*360/(self.WHEEL_DIA*math.pi))
        return 1
    
    def moveF(self, cm1 , cm2):
        self.move(cm1*360/(self.WHEEL_DIA*math.pi),cm2*360/(self.WHEEL_DIA*math.pi))
        return 1
    
    def moveBwd(self, cm):
        self.move(-cm*360/(self.WHEEL_DIA*math.pi),-cm*360/(self.WHEEL_DIA*math.pi))
        return 1
    
    def getCurrent(self):
        # gets the current of two motors in Amperes
        return (curA.read_u16()/65535,curB.read_u16()/65535)