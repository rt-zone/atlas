from machine import Pin
from encoder import Encoder as _Encoder

# TODO:
# Encoder library only accepts integer value for pins
# Encoder pins are definied in Artiware through strings 
# Temporary solution manually re-define encoder values here 
ENC_A1	= 0
ENC_A2	= 1
ENC_B1	= 3 
ENC_B2	= 2
MAX_RPS = 4 # Theoretical, imperically measuresd maximum rotations per second

def get_pins_by_letter(motor_letter):
    p1, p2 = None, None
    if motor_letter == 'A':
        p1, p2 = ENC_A1, ENC_A2
    elif motor_letter == 'B':
        p1, p2 = ENC_B1, ENC_B2
    else:
        raise ValueError(f"Invalid motor letter'{motor_letter}'. Use 'A' or 'B'.")
    return p1, p2


class Encoder(_Encoder):
    def __init__(self, motor_letter):
        self.motor_letter = motor_letter

        enc_id = 0 if motor_letter == 'A' else 1
        if motor_letter == "A":
            enc_id = 0
        elif motor_letter == "B":
            enc_id = 1
        else:
            raise ValueError (f"Wrong Encoder Letter: {motor_letter}. Use 'A' or 'B'")
        pins = get_pins_by_letter(motor_letter)
        super().__init__(0, enc_id, pins)

        self.handlers = []
        self.irq_pin = Pin(pins[0], Pin.IN)
        self.irq_pin.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler=self._handle_irq)
        self.COUNTS_PER_REV = self.counts_per_rev()

    def __call__(self):
        return self.capture()
        
    def _handle_irq(self, pin):
        for handler in self.handlers:
            handler(pin)
    
    def append_irq(self, irq_handler):
        if irq_handler not in self.handlers:
            self.handlers.append(irq_handler)
    
    def remove_irq(self, irq_handler):  
        try:
            self.handlers.remove(irq_handler)
        except ValueError:
            pass



    def get_count(self):
        return self.capture().count
    
    def get_degrees(self):
        return self.capture().degrees

    def get_revolutions(self):
        return self.capture().revolutions

    def get_speed_degrees(self):
        return self.capture().degrees_per_second
        
    def get_speed(self):
        return self.capture().revolutions_per_second / MAX_RPS