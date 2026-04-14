from machine import Pin
from encoder import Encoder

# TODO:
# Encoder library only accepts integer value for pins
# Encoder pins are definied in Artiware through strings 
# Temporary solution manually re-define encoder values here 
ENC_A1	= 0
ENC_A2	= 1
ENC_B1	= 3
ENC_B2	= 2

MAX_RPS = 4 # Theoretical, imperically measuresd maximum rotations per second

class Encoders:
    def __init__(self):
        self.right_enc = Encoder(0, 1, (ENC_A1, ENC_A2))
        self.left_enc  = Encoder(0, 0, (ENC_B1, ENC_B2))

    def __call__(self, motor):
        return self._get_encoder(motor)

    def _get_encoder(self, motor):
        target_encoder = None
        if motor == 'A':
            target_encoder = self.left_enc
        elif motor == 'B':
            target_encoder = self.right_enc
        else:
            raise ValueError(f"Invalid motor letter'{motor}'. Use 'A' or 'B'.")
        
        return target_encoder

    def get_count(self, motor):
        return self(motor).capture().count
    
    def get_degrees(self, motor):
        return self(motor).capture().degrees

    def get_rotations(self, motor):
        return self(motor).capture().revolutions

    def get_speed_degrees(self, motor):
        return self(motor).capture().degrees_per_second
        
    def get_speed_rotations(self, motor):
        return self(motor).capture().revolutions_per_second / MAX_RPS

# TODO: or not TODO, fix this pattern by providing true singleton
# Pseudo - singleton
encoders = Encoders()
