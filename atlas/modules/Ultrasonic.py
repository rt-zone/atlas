from machine import Pin, time_pulse_us
import time
class Ultrasonic:
    def __init__(self, echo_pin = "US_ECHO", trigger_pin = "US_TRIGGER"):
        self.echo =     Pin(echo_pin,       Pin.IN)
        self.trigger =  Pin(trigger_pin,    Pin.OUT)

    def __call__(self):
        return self.getDistance()

    def getDistance(self):
        """
            Returns Ultrasonic distance in cm
        """
        self.trigger.value(0)
        time.sleep_us(2)
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        duration = time_pulse_us(self.echo, 1)
        distance = (duration * 0.0343) / 2
        return distance
