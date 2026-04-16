from atlas.movement.Motor import Motor
from machine import Pin
from atlas.movement.pid import PID
import time

ma = Motor('A')
mb = Motor('B')

speed_pid = PID(kp=1.2, ki=0.12, kd=0.0, dt=0.02)

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def run_velocity_test(motor, pid):
    motor.speed_pid = pid
    test_sequence = [
        0.0,
        0.3,
        0.6,
        0.8,
        0.5,
        0.0,
        -0.5,
        -0.8,
        0.0
    ]

    print("Starting PID velocity test...")

    for target_speed in test_sequence:
        print("\n=== TARGET:", target_speed, "===\n")

        # let system settle for each step
        start_time = time.ticks_ms()

        motor.move_pid(target_speed, True)
        while time.ticks_diff(time.ticks_ms(), start_time) < 3000:  # 3 sec per step
            pass    

    motor.stop_pid()
    print("\nTest finished.")



def measure(target_speed):
    # ma.speed_pid = speed_pid
    ma.move_pid(target_speed, True)
    # mb.move_pid(target_speed)

try:
    # measure(1)
    run_velocity_test(ma, speed_pid)

    while True:
        pass
except:
    ma.move(0)
    ma.stop_pid()