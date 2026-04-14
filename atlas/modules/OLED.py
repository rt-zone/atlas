from SSD1306 import SSD1306
from machine import Pin, I2C


lines = []
LINE_HEIGHT = 10
MAX_LINES = 32 // LINE_HEIGHT  



class OLED(SSD1306):
    def __init__(self, i2c_id = 1, scl_pin = "OLED_SCL", sda_pin = "OLED_SDA"):
        super().__init__(I2C(i2c_id, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=400_000), height = 32)
        self.lines = []

    def _print_line(self, text):
        lines.append(text)
        if len(lines) > MAX_LINES:
            lines.pop(0)  # remove oldest line = scroll up effect

        self.fill(0)
        for i, line in enumerate(lines):
            self.text(line, 0, i * LINE_HEIGHT)
        self.show()

    def print(self, *args):
        max_chars = self.width // 8
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
        self.fill(0)
        self.show()