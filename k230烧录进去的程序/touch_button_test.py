"""K230 3.5-inch screen test. Does not communicate with or move the car."""
import time
from machine import TOUCH
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

sensor = Sensor()
sensor.reset()
sensor.set_framesize(width=800, height=480)
sensor.set_pixformat(Sensor.RGB565)
Display.init(Display.ST7701, to_ide=True)
MediaManager.init()
sensor.run()
touch = TOUCH(0)
result = 'TOUCH A BUTTON'
last_touch = None

try:
    while True:
        points = touch.read()
        if points:
            p = points[0]
            x, y = p.x, p.y
            now = time.ticks_ms()
            if last_touch is None or time.ticks_diff(now, last_touch) >= 500:
                last_touch = now
                if 30 <= x < 380 and 300 <= y < 450:
                    result = 'START PRESSED (TEST ONLY)'
                elif 420 <= x < 770 and 300 <= y < 450:
                    result = 'STOP PRESSED (TEST ONLY)'
                else:
                    result = 'OUTSIDE BUTTONS'
                print('touch x=%d y=%d: %s' % (x, y, result))
        img = sensor.snapshot()
        img.draw_rectangle(30, 300, 350, 150, color=(0, 130, 0), fill=True)
        img.draw_rectangle(420, 300, 350, 150, color=(180, 0, 0), fill=True)
        img.draw_string_advanced(125, 350, 32, 'START', color=(255, 255, 255))
        img.draw_string_advanced(535, 350, 32, 'STOP', color=(255, 255, 255))
        img.draw_rectangle(0, 0, 800, 48, color=(0, 0, 0), fill=True)
        img.draw_string_advanced(10, 8, 24, result, color=(255, 255, 255))
        Display.show_image(img)
except KeyboardInterrupt:
    pass
finally:
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
