"""Six-color YOLO comparison. Separate model paths prevent ring/color collision.
Same inference size, thresholds, labels and display as the supplied demo.
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

MODEL='/sdcard/codex_yolo/colors.kmodel'
LABELS=['black','deep blue','green','light blue','red','yellow']

class ColorYOLO(YOLO11):
    def postprocess(self,results):
        # Check BEFORE calling the native decoder with a class count.
        shape=results[0].shape
        if len(shape)!=3 or shape[0]!=1 or shape[1]!=4+len(LABELS):
            raise ValueError('Six-color model output must be [1,10,N], got '+str(shape))
        return super().postprocess(results)

def main():
    pl=None
    yolo=None
    frames=0
    started=time.ticks_ms()
    window_start=started
    window_frames=0
    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        print('COLOR_STAGE','camera')
        pl=PipeLine(rgb888p_size=[640,360],display_size=[800,480],display_mode='st7701')
        pl.create(sensor=Sensor(id=2,width=1920,height=1080))
        print('COLOR_STAGE','load_model',MODEL,os.stat(MODEL)[6])
        yolo=ColorYOLO(task_type='detect',mode='video',kmodel_path=MODEL,
            labels=LABELS,rgb888p_size=[640,360],model_input_size=[320,320],
            display_size=pl.get_display_size(),conf_thresh=0.6,nms_thresh=0.45,
            max_boxes_num=50,debug_mode=0)
        yolo.config_preprocess()
        print('COLOR_READY','six classes; conf=0.6; no UART')
        while True:
            os.exitpoint()
            img=pl.get_frame()
            res=yolo.run(img)
            yolo.draw_result(res,pl.osd_img)
            pl.show_image()
            frames+=1
            window_frames+=1
            gc.collect()
            now=time.ticks_ms()
            elapsed=time.ticks_diff(now,window_start)
            if elapsed>=1000:
                detections=[(LABELS[int(res[1][i])],round(float(res[2][i]),3))
                            for i in range(len(res[1]))] if res else []
                print('COLOR_HEARTBEAT',{'frames':frames,
                    'uptime_s':time.ticks_diff(now,started)//1000,
                    'fps':round(window_frames*1000/elapsed,1),'detections':detections})
                window_start=now
                window_frames=0
            # Give the IDE transport time; no full result dump every frame.
            time.sleep_ms(5)
    except KeyboardInterrupt:
        print('COLOR_STOPPED')
    except Exception as error:
        if str(error)=='IDE interrupt':
            print('COLOR_STOPPED')
        else:
            print('COLOR_ERROR',repr(error))
            sys.print_exception(error)
            raise
    finally:
        if yolo is not None:
            try:yolo.deinit()
            except Exception as error:print('COLOR_CLEANUP_MODEL',repr(error))
        if pl is not None:
            try:
                if pl.sensor is not None:pl.sensor.stop()
            except Exception as error:print('COLOR_CLEANUP_SENSOR',repr(error))
            try:Display.deinit()
            except Exception as error:print('COLOR_CLEANUP_DISPLAY',repr(error))
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        try:MediaManager.deinit()
        except Exception as error:print('COLOR_CLEANUP_MEDIA',repr(error))

if __name__=='__main__':main()

