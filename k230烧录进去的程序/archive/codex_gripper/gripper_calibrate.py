# Motion disabled after reported mechanical damage. Recalibrate unloaded before restoring.
raise RuntimeError('GRIPPER_MOTION_LOCKED: detach damaged mechanism and inspect before any motor test')
"""Manual visual calibration; no automatic grasp. Click the preview buttons.
After eight seconds without input the PWM is disabled. A saved pulse is only
a jaw position: it does not verify grip force or the available power current.
"""
import os
import sys
import time
import ujson
from machine import FPIOA, Pin, PWM, TOUCH

sys.path.insert(0, '/sdcard/codex_gripper')
from color_gripper_preview import main

CALIBRATION = '/sdcard/codex_gripper/jaw_calibration.json'


class JawCalibration:
    def __init__(self):
        self.fpioa = FPIOA()
        self.pwm = None
        self.current = 1500
        self.target = 1500
        self.last_step = time.ticks_ms()
        self.last_input = self.last_step
        self.open_us = None
        self.close_us = None
        self.last_tap = time.ticks_add(self.last_step, -1000)
        self.off()
        self.touch = TOUCH(TOUCH.DEV_IDE, range_x=640, range_y=360)
        print('CAL_READY: output OFF; each +/- click moves 50 us slowly')

    def off(self):
        if self.pwm is not None:
            self.pwm.deinit()
            self.pwm = None
        self.fpioa.set_function(42, FPIOA.GPIO42)
        Pin(42, Pin.OUT, value=0)
        self.target = self.current

    def move(self, delta):
        # These are software bounds, NOT verified mechanical endpoints.
        target = self.current + delta
        if not 700 <= target <= 2300:
            print('CAL_LIMIT: requested pulse outside 700..2300 us')
            return
        if self.pwm is None:
            self.fpioa.set_function(42, FPIOA.PWM0)
            self.pwm = PWM(0, freq=50, duty_ns=self.current * 1000)
        self.target = target
        self.last_input = time.ticks_ms()
        print('CAL_MOVE', self.current, 'to', self.target)

    def save(self, is_open):
        if self.pwm is None or self.current != self.target:
            print('CAL_SAVE_REJECTED: wait for movement, then save while ON')
            return
        if is_open:
            self.open_us = self.current
        else:
            self.close_us = self.current
        data = {'pin': 42, 'freq': 50, 'open_us': self.open_us,
                'close_us': self.close_us, 'auto_grasp_enabled': False}
        with open(CALIBRATION, 'w') as f:
            ujson.dump(data, f)
        self.last_input = time.ticks_ms()
        print('CAL_SAVED', data)

    def poll(self):
        now = time.ticks_ms()
        for point in self.touch.read(5):
            if point.event != TOUCH.EVENT_DOWN or not 315 <= point.y < 360:
                continue
            if time.ticks_diff(now, self.last_tap) < 350:
                continue
            self.last_tap = now
            x = point.x
            if x < 100:
                self.off()
                print('CAL_OUTPUT_OFF', self.current)
            elif self.current != self.target:
                print('CAL_BUSY: wait or use OFF')
            elif x < 220:
                self.move(-50)
            elif x < 340:
                self.move(50)
            elif x < 490:
                self.save(True)
            else:
                self.save(False)
        if self.pwm is not None:
            if time.ticks_diff(now, self.last_input) >= 8000:
                self.off()
                print('CAL_IDLE_OUTPUT_OFF', self.current)
            elif self.current != self.target and time.ticks_diff(now, self.last_step) >= 20:
                self.current += 1 if self.target > self.current else -1
                self.pwm.duty_ns(self.current * 1000)
                self.last_step = now
                if self.current == self.target:
                    print('CAL_REACHED_US', self.current)

    def draw(self, img):
        img.draw_string_advanced(5, 28, 18,
            'us:%d -> %d %s open:%s close:%s' %
            (self.current, self.target, 'ON' if self.pwm else 'OFF',
             str(self.open_us), str(self.close_us)), color=(255, 255, 0))
        for left, width, label in ((0, 100, 'OFF'), (100, 120, '-50us'),
                                   (220, 120, '+50us'), (340, 150, 'SET OPEN'),
                                   (490, 150, 'SET CLOSED')):
            img.draw_rectangle(left, 315, width-2, 44, color=(30, 30, 30), fill=True)
            img.draw_string_advanced(left+6, 327, 17, label, color=(255, 255, 255))

    def close(self):
        self.off()
        self.touch.deinit()


if __name__ == '__main__':
    os.exitpoint(os.EXITPOINT_ENABLE)
    main(JawCalibration())