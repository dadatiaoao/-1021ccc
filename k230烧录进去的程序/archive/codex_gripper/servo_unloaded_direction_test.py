# Disabled after unexpected delayed motion; physical position is unknown.
raise RuntimeError('GRIPPER_MOTION_LOCKED: unexpected delayed motion; inspect unloaded hardware before resuming')
"""Unloaded servo direction check only. Do not attach the damaged gripper.
This does not unlock the earlier gripper calibration or startup scripts.
"""
import os
import time
from machine import FPIOA, Pin, PWM

def wait_ms(duration):
    deadline = time.ticks_add(time.ticks_ms(), duration)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        os.exitpoint()
        time.sleep_ms(20)

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    fpioa = FPIOA()
    pwm = None
    try:
        print('UNLOADED_STAGE_1_BASELINE_1500: wait 3 seconds')
        fpioa.set_function(42, FPIOA.PWM0)
        pwm = PWM(0, freq=50, duty_ns=1500000)
        wait_ms(3000)
        print('UNLOADED_STAGE_2_INCREASE_1500_TO_1600: observe direction NOW')
        for pulse in range(1501, 1601):
            os.exitpoint()
            pwm.duty_ns(pulse * 1000)
            time.sleep_ms(20)
        print('UNLOADED_STAGE_3_HOLD_1600: wait 3 seconds')
        wait_ms(3000)
        print('UNLOADED_STAGE_4_DECREASE_1600_TO_1500')
        for pulse in range(1599, 1499, -1):
            os.exitpoint()
            pwm.duty_ns(pulse * 1000)
            time.sleep_ms(20)
        wait_ms(500)
    finally:
        if pwm is not None:
            pwm.deinit()
        fpioa.set_function(42, FPIOA.GPIO42)
        pin = Pin(42, Pin.OUT, value=0)
        print('UNLOADED_OUTPUT_OFF', pin.value())
    print('UNLOADED_TEST_DONE: physical direction requires observation')

main()
