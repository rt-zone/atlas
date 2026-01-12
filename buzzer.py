# Robo-speech for a piezo buzzer (MicroPython, Raspberry Pi Pico / Pico W)
# - Each character becomes a tiny "phoneme" made from beeps + quick frequency slides.
# - Edit BUZZER_PIN to your wiring.

from machine import Pin, PWM
from time import sleep_ms

BUZZER_PIN = 18  # change to your buzzer pin (e.g. 15, 16, etc.)
buzzer = PWM(Pin(BUZZER_PIN))
buzzer.duty_u16(0)

# --- core sound helpers -------------------------------------------------------

def tone(freq_hz: int, ms: int, vol=18000):
    """Play a steady tone."""
    if freq_hz <= 0:
        buzzer.duty_u16(0)
        sleep_ms(ms)
        return
    buzzer.freq(freq_hz)
    buzzer.duty_u16(vol)
    sleep_ms(ms)
    buzzer.duty_u16(0)

def slide(f0: int, f1: int, ms: int, steps=10, vol=18000):
    """Quick linear pitch slide (speech-ish formant feel)."""
    if f0 <= 0 or f1 <= 0:
        buzzer.duty_u16(0)
        sleep_ms(ms)
        return
    buzzer.duty_u16(vol)
    dt = max(1, ms // steps)
    for i in range(steps):
        f = f0 + (f1 - f0) * i // (steps - 1)
        buzzer.freq(int(f))
        sleep_ms(dt)
    buzzer.duty_u16(0)

def trill(base: int, spread: int, ms: int, rate=12, vol=16000):
    """Fast alternating tones to imitate consonants like R/Z/S."""
    if base <= 0:
        sleep_ms(ms)
        return
    buzzer.duty_u16(vol)
    dt = max(1, ms // rate)
    for i in range(rate):
        buzzer.freq(base + (spread if i % 2 else -spread))
        sleep_ms(dt)
    buzzer.duty_u16(0)

# --- "phoneme" patterns -------------------------------------------------------
# Each entry is a list of actions: ("tone"/"slide"/"trill"/"pause", params...)
# Tune these to taste. Short, punchy patterns sound more "talky".
PHONEMES = {
    "A": [("slide", 800, 1200, 70), ("tone", 1100, 40)],
    "E": [("slide", 1100, 900, 70), ("tone", 950, 40)],
    "I": [("tone", 1400, 35), ("slide", 1400, 1700, 45)],
    "O": [("slide", 700, 500, 90), ("tone", 520, 50)],
    "U": [("tone", 600, 45), ("slide", 600, 420, 70)],

    "B": [("tone", 350, 25), ("pause", 15), ("tone", 900, 45)],
    "C": [("tone", 1100, 20), ("pause", 10), ("tone", 900, 30)],
    "D": [("tone", 450, 25), ("pause", 10), ("tone", 850, 45)],
    "F": [("trill", 1200, 120, 70), ("tone", 900, 25)],
    "G": [("tone", 500, 25), ("slide", 700, 900, 60)],
    "H": [("trill", 1000, 80, 60)],
    "J": [("tone", 900, 20), ("slide", 900, 1300, 55)],
    "K": [("tone", 1300, 18), ("pause", 10), ("tone", 700, 28)],
    "L": [("slide", 900, 700, 55), ("tone", 720, 35)],
    "M": [("tone", 300, 80)],
    "N": [("tone", 340, 60), ("tone", 520, 25)],
    "P": [("tone", 420, 22), ("pause", 10), ("tone", 1050, 40)],
    "Q": [("tone", 750, 25), ("slide", 750, 1150, 55)],
    "R": [("trill", 900, 160, 80), ("tone", 820, 25)],
    "S": [("trill", 1400, 200, 90)],
    "T": [("tone", 1200, 18), ("pause", 10), ("tone", 800, 25)],
    "V": [("trill", 1000, 140, 75)],
    "W": [("tone", 500, 25), ("slide", 500, 800, 80)],
    "X": [("tone", 1300, 18), ("pause", 10), ("trill", 1200, 180, 55)],
    "Y": [("tone", 1000, 20), ("slide", 1000, 1500, 60)],
    "Z": [("trill", 1100, 220, 90)],

    " ": [("pause", 120)],
    ".": [("tone", 600, 40), ("pause", 100)],
    ",": [("tone", 700, 25), ("pause", 70)],
    "?": [("slide", 900, 1400, 110), ("pause", 80)],
    "!": [("tone", 1400, 40), ("tone", 1400, 40), ("pause", 80)],
}

def play_action(action):
    kind = action[0]
    if kind == "tone":
        _, f, ms = action
        tone(f, ms)
    elif kind == "slide":
        _, f0, f1, ms = action
        slide(f0, f1, ms)
    elif kind == "trill":
        _, base, spread, ms = action
        trill(base, spread, ms)
    elif kind == "pause":
        _, ms = action
        tone(0, ms)


def speak(text: str, char_gap=25):
    """Speak text with buzzer-phonemes."""
    text = text.upper()
    for ch in text:
        pattern = PHONEMES.get(ch)
        if pattern is None:
            # Unknown char -> small click
            tone(1000, 10)
            tone(0, 25)
        else:
            for act in pattern:
                play_action(act)
        tone(0, char_gap)

def say_word(word: str):
    # A slightly "sentence-like" intonation: small upshift at the end
    speak(word, char_gap=18)
    slide(700, 1000, 120)
    tone(0, 120)

# --- demo --------------------------------------------------------------------
try:
    say_word("HELLO")
finally:
    buzzer.duty_u16(0)
    buzzer.deinit()
