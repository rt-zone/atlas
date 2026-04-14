from machine import Pin 

class LineSensor:
    def __init__(self, is_irq=False):
        self.left_pin   = Pin("LINE_LEFT", Pin.IN)
        self.mid_pin    = Pin("LINE_MID", Pin.IN) 
        self.right_pin  = Pin("LINE_RIGHT", Pin.IN)
        self.is_irq = is_irq
        
        if is_irq:
            self._setup_irq()
            
    def _setup_irq(self):
        self.left_irq_val = 0
        self.mid_irq_val = 0
        self.right_irq_val = 0

        def handle(pin, attr):
            setattr(self, attr, pin.value())

        self.left_pin.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=lambda pin: handle(pin, 'left_irq_val'))
        self.mid_pin.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=lambda pin: handle(pin, 'mid_irq_val'))
        self.right_pin.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=lambda pin: handle(pin, 'right_irq_val'))

    def getLeft(self):
        return self.left_pin.value() if not self.is_irq else self.left_irq_val
    
    def getMiddle(self):
        return self.mid_pin.value() if not self.is_irq else self.mid_irq_val
    
    def getRight(self):
        return self.right_pin.value() if not self.is_irq else self.right_irq_val
