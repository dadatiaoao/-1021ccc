# Disabled after unexpected delayed motion; physical position is unknown.
raise RuntimeError('GRIPPER_MOTION_LOCKED: unexpected delayed motion; inspect unloaded hardware before resuming')
"""Visual unloaded test only. The gripper mechanism must be detached."""
import os
import sys
import time
from machine import FPIOA, Pin, PWM, TOUCH
sys.path.insert(0, '/sdcard/codex_gripper')
from color_gripper_preview import main

class UnloadedVisualTest:
    def __init__(self):
        self.fpioa = FPIOA()
        self.pwm = None
        self.us = 1500
        self.stage = 'START'
        self.since = time.ticks_ms()
        self.last_step = self.since
        self.off()
        self.touch = TOUCH(TOUCH.DEV_IDE, range_x=640, range_y=360)

    def off(self):
        if self.pwm is not None:
            self.pwm.deinit()
            self.pwm = None
        self.fpioa.set_function(42, FPIOA.GPIO42)
        Pin(42, Pin.OUT, value=0)

    def transition(self, stage, now):
        self.stage = stage
        self.since = now
        self.last_step = now
        print('UNLOADED_VISUAL', stage, self.us)

    def poll(self):
        now = time.ticks_ms()
        for point in self.touch.read(5):
            if point.event == TOUCH.EVENT_DOWN and point.y >= 310:
                self.off()
                self.transition('STOPPED_OUTPUT_OFF', now)
        if self.stage == 'START':
            self.fpioa.set_function(42, FPIOA.PWM0)
            self.pwm = PWM(0, freq=50, duty_ns=1500000)
            self.transition('BASELINE_WAIT_10S', now)
        elif self.stage == 'BASELINE_WAIT_10S':
            if time.ticks_diff(now, self.since) >= 10000:
                self.transition('INCREASE_1500_TO_1600', now)
        elif self.stage == 'INCREASE_1500_TO_1600':
            if time.ticks_diff(now, self.last_step) >= 20:
                self.us = min(1600, self.us + 2)
                self.pwm.duty_ns(self.us * 1000)
                self.last_step = now
                if self.us == 1600:
                    self.transition('HOLD_1600_10S', now)
        elif self.stage == 'HOLD_1600_10S':
            if time.ticks_diff(now, self.since) >= 10000:
                self.transition('RETURN_1600_TO_1500', now)
        elif self.stage == 'RETURN_1600_TO_1500':
            if time.ticks_diff(now, self.last_step) >= 20:
                self.us = max(1500, self.us - 2)
                self.pwm.duty_ns(self.us * 1000)
                self.last_step = now
                if self.us == 1500:
                    self.off()
                    self.transition('DONE_OUTPUT_OFF', now)

    def draw(self, img):
        img.draw_rectangle(0, 0, 640, 60, color=(20, 20, 20), fill=True)
        img.draw_string_advanced(6, 4, 20, 'UNLOADED SERVO TEST - NO GRIPPER', color=(0,255,255))
        img.draw_string_advanced(6, 30, 19, '%s  %dus' % (self.stage, self.us), color=(255,255,0))
        img.draw_rectangle(0,310,640,50,color=(120,0,0),fill=True)
        img.draw_string_advanced(160,323,20,'STOP / OUTPUT OFF',color=(255,255,255))

    def close(self):
        self.off()
        self.touch.deinit()

if __name__ == '__main__':
    os.exitpoint(os.EXITPOINT_ENABLE)
    main(UnloadedVisualTest())
