# Disabled after unexpected delayed motion; physical position is unknown.
raise RuntimeError('GRIPPER_MOTION_LOCKED: unexpected delayed motion; inspect unloaded hardware before resuming')
"""Operator-controlled 10 us jaw trim; no automatic centering or grasp.
Requires the operator to confirm the last commanded position is unchanged.
STOP disables PWM only; disconnect external servo power if it binds.
"""
import os
import sys
import time
import ujson
from machine import FPIOA, Pin, PWM, TOUCH
sys.path.insert(0, '/sdcard/codex_gripper')
from color_gripper_preview import main

STATE = '/sdcard/codex_gripper/manual_10us_state.json'
LOW_US, HIGH_US = 1400, 1600  # Exploration window, NOT measured mechanical limits.

class ManualJaw:
    def __init__(self):
        with open(STATE) as f:
            self.current = int(ujson.load(f)['last_command_us'])
        if not LOW_US <= self.current <= HIGH_US:
            raise ValueError('Manual starting command outside review window')
        self.fpioa = FPIOA()
        self.pwm = None
        self.target = self.current
        self.last_step = time.ticks_ms()
        self.reached_at = self.last_step
        self.last_tap = time.ticks_add(self.last_step, -2000)
        self.input_ready_at = None
        self.status = 'OFF - waiting for your single click'
        self.output_low()
        self.touch = TOUCH(TOUCH.DEV_IDE, range_x=640, range_y=360)
        print('MANUAL_READY_NO_OUTPUT', self.current)

    def output_low(self):
        if self.pwm is not None:
            self.pwm.deinit()
            self.pwm = None
        self.fpioa.set_function(42, FPIOA.GPIO42)
        Pin(42, Pin.OUT, value=0)

    def stop(self, message):
        self.output_low()
        self.target = self.current
        self.status = message
        with open(STATE, 'w') as f:
            ujson.dump({'last_command_us': self.current, 'measured': False}, f)
        print('MANUAL_PWM_OFF', self.current, message)

    def step(self, delta):
        if self.pwm is not None:
            return
        target = self.current + delta
        if not LOW_US <= target <= HIGH_US:
            self.status = 'WINDOW END - stop and review'
            print('MANUAL_WINDOW_END', self.current)
            return
        self.fpioa.set_function(42, FPIOA.PWM0)
        self.pwm = PWM(0, freq=50, duty_ns=self.current * 1000)
        self.target = target
        self.last_step = time.ticks_ms()
        self.status = 'ONE STEP - observe the jaw'
        print('MANUAL_ONE_STEP', self.current, 'to', self.target)

    def poll(self):
        now = time.ticks_ms()
        if self.input_ready_at is None:
            self.input_ready_at = time.ticks_add(now, 1000)
        for point in self.touch.read(5):
            if point.event != TOUCH.EVENT_DOWN or point.y < 310:
                continue
            if point.x < 280:  # STOP is never blocked by debounce.
                self.stop('OFF - operator stop')
            elif time.ticks_diff(now, self.input_ready_at) < 0:
                continue  # Drain events left over before this panel was ready.
            elif time.ticks_diff(now, self.last_tap) >= 800:
                self.last_tap = now
                if self.pwm is None:
                    self.step(-10 if point.x < 460 else 10)
        if self.pwm is not None:
            if self.current != self.target:
                if time.ticks_diff(now, self.last_step) >= 25:
                    self.current += 1 if self.target > self.current else -1
                    self.pwm.duty_ns(self.current * 1000)
                    self.last_step = now
                    if self.current == self.target:
                        self.reached_at = now
                        print('MANUAL_REACHED', self.current)
            elif time.ticks_diff(now, self.reached_at) >= 700:
                self.stop('OFF - one step finished')

    def draw(self, img):
        img.draw_rectangle(0, 0, 640, 62, color=(20,20,20), fill=True)
        img.draw_string_advanced(6, 3, 20, 'MANUAL ONLY - ONE CLICK = 10us', color=(0,255,255))
        img.draw_string_advanced(6, 29, 18, '%dus -> %dus | %s' %
            (self.current, self.target, self.status), color=(255,255,0))
        for left, width, label, color in (
            (0,280,'STOP PWM / CUT POWER IF BINDING',(120,0,0)),
            (280,180,'-10us',(25,50,70)),
            (460,180,'+10us',(25,50,70))):
            img.draw_rectangle(left,310,width-2,50,color=color,fill=True)
            img.draw_string_advanced(left+5,326,16,label,color=(255,255,255))

    def close(self):
        self.stop('OFF - script stopped')
        self.touch.deinit()

if __name__ == '__main__':
    os.exitpoint(os.EXITPOINT_ENABLE)
    main(ManualJaw())