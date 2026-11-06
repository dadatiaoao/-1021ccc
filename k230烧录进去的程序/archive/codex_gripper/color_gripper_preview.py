"""Six-color preview for gripper calibration. This script NEVER drives a servo.
Model: user's 15131 export, SHA256
437c4ea392055125a036319639329ca4e538811a454997d99f29ee331280ada3.
The cyan box is a provisional image region, NOT a calibrated gripper position.
"""
import os
import sys
import time
import gc
from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

MODEL = '/sdcard/codex_gripper/colors_15131.kmodel'
LABELS = ['black', 'deep blue', 'green', 'light blue', 'red', 'yellow']
WIDTH, HEIGHT = 640, 360


class ColorYOLO(YOLO11):
    def postprocess(self, results):
        shape = results[0].shape
        if len(shape) != 3 or shape[0] != 1 or shape[1] != 10:
            raise ValueError('Expected six-color output [1,10,N], got ' + str(shape))
        return super().postprocess(results)


def main(controls=None):
    pl = None
    yolo = None
    frames = 0
    last_log = time.ticks_ms()
    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        print('GRIPPER_VISION_START', 'manual calibration' if controls else 'preview only, NO servo commands')
        pl = PipeLine(rgb888p_size=[640, 360], display_size=[WIDTH, HEIGHT],
                      display_mode='virt')
        pl.create(sensor=Sensor(id=2, width=1920, height=1080))
        yolo = ColorYOLO(task_type='detect', mode='video', kmodel_path=MODEL,
                         labels=LABELS, rgb888p_size=[640, 360],
                         model_input_size=[320, 320], display_size=pl.get_display_size(),
                         conf_thresh=0.6, nms_thresh=0.45, max_boxes_num=20,
                         debug_mode=0)
        yolo.config_preprocess()
        print('GRIPPER_VISION_READY', MODEL)
        while True:
            os.exitpoint()
            result = yolo.run(pl.get_frame())
            yolo.draw_result(result, pl.osd_img)
            pl.osd_img.draw_rectangle(280, 140, 80, 80, color=(0, 255, 255), thickness=2)
            title = 'MANUAL CALIBRATION' if controls else 'PREVIEW ONLY - NO GRIP'
            pl.osd_img.draw_string_advanced(5, 5, 20, title, color=(0, 255, 255))
            detections = []
            if result:
                for i in range(len(result[1])):
                    x, y, w, h = [int(round(v)) for v in result[0][i]]
                    cx, cy = x + w // 2, y + h // 2
                    pl.osd_img.draw_cross(cx, cy, color=(255, 255, 255), size=8)
                    detections.append((LABELS[int(result[1][i])],
                                       round(float(result[2][i]), 3), cx, cy, w, h))
            if controls is not None:
                controls.poll()
                controls.draw(pl.osd_img)
            pl.show_image()
            frames += 1
            now = time.ticks_ms()
            if time.ticks_diff(now, last_log) >= 1000:
                print('GRIPPER_VISION', frames, detections)
                last_log = now
            gc.collect()
            time.sleep_ms(5)
    except KeyboardInterrupt:
        print('GRIPPER_VISION_STOPPED')
    except Exception as error:
        if str(error) != 'IDE interrupt':
            sys.print_exception(error)
            raise
    finally:
        if controls is not None:
            controls.close()
        if yolo is not None:
            yolo.deinit()
        if pl is not None:
            pl.destroy()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == '__main__':
    main()