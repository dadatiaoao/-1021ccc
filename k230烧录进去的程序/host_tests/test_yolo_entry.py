"""Host checks for the real entry/app loop, using no camera, KPU or UART.
Run with CPython: python -m unittest discover -s host_tests -v
"""
from contextlib import redirect_stdout
import importlib.util
import io
from pathlib import Path
import runpy
import sys
import types
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
PACKAGE = Path(__file__).resolve().parents[1]/'transport_vision'
CARD = PACKAGE.parent
sys.path.insert(0, str(PACKAGE))
import vision_config as cfg
import vision_logic as logic
import vision_protocol as wire


class Runtime:
    def __init__(self, frames=4, commands=None):
        self.frames = frames
        self.commands = commands or {}
        self.iteration = 0
        self.poll = 0
        self.now = 0
        self.model_runs = []
        self.models = []
        self.packets = []
        self.uart_args = []
        self.sensor_stopped = False
        self.uart_closed = False
        self.osd_frames = []
        self.refine_calls = []
        self.refine_failure = False
        class Tensor:
            def __init__(self): self.planes = [object(),object(),object()]
            def __getitem__(self,index): return self.planes[index]
        self.tensor = Tensor()

    def ticks_ms(self):
        self.now += 25
        return self.now

    def exitpoint(self, *args):
        if args:
            return
        self.iteration += 1
        self.poll = 0
        if self.iteration > self.frames:
            raise KeyboardInterrupt

    def stat(self, path):
        return (CARD / path.removeprefix('/sdcard/')).stat()

    def modules(self):
        runtime = self

        class YOLO:
            def __init__(self, **kwargs):
                self.labels = kwargs['labels']
                self.path = kwargs['kmodel_path']
                self.closed = False
                runtime.models.append(self)

            def config_preprocess(self):
                pass

            def postprocess(self, result):
                return result

            def run(self, tensor):
                runtime.model_runs.append((runtime.iteration, self.path, tensor))
                if len(self.labels) == 6:
                    return ([[20+60*i, 50, 20, 20] for i in range(6)],
                            list(range(6)), [0.9]*6)
                return ([[50,20,80,80],[170,20,80,80],[350,20,80,80]],
                        [0,1,2], [0.9]*3)

            def deinit(self):
                self.closed = True

        class Sensor:
            YUV420SP, RGB565, RGBP888 = 1, 2, 3

            def __init__(self, **kwargs):
                pass

            def reset(self): pass
            def set_hmirror(self, value): pass
            def set_vflip(self, value): pass
            def set_framesize(self, **kwargs): pass
            def set_pixformat(self, *args, **kwargs): pass
            def bind_info(self, **kwargs): return kwargs
            def run(self): pass

            def snapshot(self, **kwargs):
                return types.SimpleNamespace(
                    to_numpy_ref=lambda: runtime.tensor,
                    find_qrcodes=lambda: [types.SimpleNamespace(payload=lambda: '156+123+516+231')])

            def stop(self):
                runtime.sensor_stopped = True

        class Image:
            def __init__(self, *args, **kwargs):
                self.labels = []
                self.plane = kwargs.get('data')

            def clear(self): self.labels = []
            def draw_rectangle(self, *args, **kwargs): pass
            def draw_cross(self, *args, **kwargs): pass
            def draw_string_advanced(self, x, y, size, text, **kwargs):
                self.labels.append(text)
            def find_circles(self, **kwargs):
                runtime.refine_calls.append((self.plane,kwargs))
                if runtime.refine_failure:raise ValueError('Unsupported image API')
                return [types.SimpleNamespace(x=lambda:210,y=lambda:60,r=lambda r=r:r)
                        for r in (10,20,30)]

        class Display:
            VIRT, LAYER_VIDEO1, LAYER_OSD3 = 1, 2, 3
            @staticmethod
            def init(*args, **kwargs): pass
            @staticmethod
            def bind_layer(**kwargs): pass
            @staticmethod
            def show_image(image, *args):
                runtime.osd_frames.append(list(image.labels))
            @staticmethod
            def deinit(): pass

        class MediaManager:
            @staticmethod
            def init(): pass
            @staticmethod
            def deinit(): pass

        class FPIOA:
            UART1_TXD, UART1_RXD, UART2_TXD, UART2_RXD = 1, 2, 3, 4
            def set_function(self, *args): pass

        # Deliberately no EIGHTBITS/PARITY_NONE/STOPBITS_ONE attributes.
        class UART:
            def __init__(self, *args, **kwargs):
                runtime.uart_args.append((args, kwargs))

            def read(self,*args):
                runtime.poll += 1
                return runtime.commands.get((runtime.iteration, runtime.poll))

            def write(self, packet):
                runtime.packets.append(packet)
                return len(packet)

            def deinit(self):
                runtime.uart_closed = True

        def module(name, **attributes):
            item = types.ModuleType(name)
            item.__dict__.update(attributes)
            return item

        return {
            'libs': module('libs'), 'libs.YOLO': module('libs.YOLO', YOLO11=YOLO),
            'media': module('media'), 'media.sensor': module('media.sensor', Sensor=Sensor),
            'media.display': module('media.display', Display=Display),
            'media.media': module('media.media', MediaManager=MediaManager),
            'image': module('image', Image=Image, ARGB8888=1,GRAYSCALE=2,ALLOC_REF=3),
            'machine': module('machine', FPIOA=FPIOA, UART=UART),
        }

    def app(self):
        with patch.dict(sys.modules, self.modules()):
            spec = importlib.util.spec_from_file_location('test_real_vision_app', PACKAGE/'vision_app.py')
            app = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(app)
        app.os = types.SimpleNamespace(exitpoint=self.exitpoint, stat=self.stat,
                                       EXITPOINT_ENABLE=1, EXITPOINT_ENABLE_SLEEP=2)
        app.time = types.SimpleNamespace(ticks_ms=self.ticks_ms,
                                         ticks_diff=lambda a,b:a-b, sleep_ms=lambda ms:None)
        app.gc = types.SimpleNamespace(collect=lambda:None)
        app.sys = types.SimpleNamespace(print_exception=lambda error:None)
        return app

    def run(self, **kwargs):
        app = self.app()
        with patch.dict(sys.modules, self.modules()), redirect_stdout(io.StringIO()):
            app.run(**kwargs)

    def decoded(self):
        return wire.Parser().feed(b''.join(self.packets))


class YoloEntryTests(unittest.TestCase):
    def test_boot_dispatches_to_yolo_integrated_app(self):
        calls = []
        stub = types.ModuleType('vision_app')
        stub.run = lambda **kwargs:calls.append(kwargs)
        with patch.dict(sys.modules, {'vision_app':stub}), patch.object(sys, 'path', list(sys.path)):
            runpy.run_path(str(CARD/'main.py'), run_name='__main__')
        self.assertEqual(calls, [{'bench':False}])

    def test_startup_previews_both_models_without_observation_reports(self):
        runtime = Runtime(frames=3)
        runtime.run()
        self.assertEqual(len(runtime.model_runs), 6)
        for index in range(0, 6, 2):
            first, second = runtime.model_runs[index:index+2]
            self.assertEqual(first[1], cfg.COLOR_MODEL)
            self.assertEqual(second[1], cfg.RING_MODEL)
            self.assertIs(first[2], second[2])
        self.assertTrue(all('YOLO PREVIEW / WAIT HOST' in labels for labels in runtime.osd_frames))
        self.assertTrue(any(any(label.startswith('C6 ') for label in labels) for labels in runtime.osd_frames))
        self.assertTrue(any(any(label.startswith('R2 ') for label in labels) for labels in runtime.osd_frames))
        # Cleanup revokes observations with NOTFOUND; startup itself sends no targets.
        self.assertTrue(all(cmd in (wire.HEARTBEAT, wire.NOTFOUND) for cmd,_ in runtime.decoded()))
        self.assertEqual(runtime.uart_args, [((cfg.UART_ID,cfg.UART_BAUDRATE),{})])
        self.assertTrue(runtime.sensor_stopped and runtime.uart_closed)
        self.assertTrue(all(model.closed for model in runtime.models))

    def test_host_selects_real_color_class(self):
        command = wire.frame(wire.SET_COLOR,[6]) + wire.frame(wire.SCAN,[1])
        runtime = Runtime(commands={(1,1):command})
        runtime.run()
        found = [params for cmd,params in runtime.decoded() if cmd == wire.FOUND]
        self.assertTrue(found)
        self.assertTrue(all(params[0] == 6 for params in found))
        self.assertTrue(all(path == cfg.COLOR_MODEL for _,path,_ in runtime.model_runs))

    def test_ring_class_and_stable_place_use_both_models(self):
        command = (wire.frame(wire.SET_COLOR,[6]) + wire.frame(wire.SET_TARGET,[1,2]) +
                   wire.frame(wire.SCAN,[2]))
        runtime = Runtime(frames=5,commands={(1,1):command})
        runtime.run()
        rings = [params for cmd,params in runtime.decoded() if cmd == wire.RING]
        self.assertGreaterEqual(len(rings),3)
        self.assertTrue(all(params == bytes([2])+wire.u16(420)+wire.u16(120) for params in rings))
        places = [params for cmd,params in runtime.decoded() if cmd == wire.PLACE_CHECK]
        # Uncalibrated geometry must not certify a correct competition placement.
        self.assertTrue(all(params == bytes([1,2,0]) for params in places))
        self.assertEqual(len(runtime.model_runs),10)

    def test_stop_during_inference_discards_frame_and_does_not_restart_preview(self):
        runtime = Runtime(frames=3,commands={
            (1,1):wire.frame(wire.SCAN,[2]), (1,2):wire.frame(wire.STOP),
            (2,1):wire.frame(wire.SET_COLOR,[6])})
        runtime.run()
        self.assertEqual([iteration for iteration,_,_ in runtime.model_runs],[1,1])
        self.assertFalse(any(cmd in (wire.FOUND,wire.RING,wire.PLACE_CHECK) for cmd,_ in runtime.decoded()))
        self.assertTrue(all(labels == ['mode=-1 color=6 ring=1'] for labels in runtime.osd_frames[1:]))

    def test_explicit_preview_never_opens_uart(self):
        runtime = Runtime(frames=2)
        runtime.run(bench=True)
        self.assertEqual(len(runtime.model_runs),4)
        self.assertEqual(runtime.uart_args,[])
        self.assertEqual(runtime.packets,[])

    def test_qr_still_reports_valid_task(self):
        runtime = Runtime(commands={(1,1):wire.frame(wire.SCAN,[0])})
        runtime.run()
        tasks = [params for cmd,params in runtime.decoded() if cmd == wire.TASKCODE]
        self.assertEqual(tasks,[logic.parse_task('156+123+516+231')])
        self.assertEqual(runtime.model_runs,[])

    def test_wrong_model_shape_is_rejected_before_native_decoder(self):
        app = Runtime().app()
        model = app.CheckedYOLO.__new__(app.CheckedYOLO)
        model.labels = cfg.COLOR_LABELS
        good = [types.SimpleNamespace(shape=(1,10,2100))]
        self.assertIs(model.postprocess(good),good)
        with self.assertRaises(ValueError):
            model.postprocess([types.SimpleNamespace(shape=(1,7,2100))])

    def test_precision_mode_refines_the_same_frame_inside_yolo_roi(self):
        command = (wire.frame(wire.SET_TARGET,[1,2])+wire.frame(wire.SET_CONTEXT,[0,7,0,3])+
                   wire.frame(wire.SCAN,[4]))
        runtime = Runtime(frames=2,commands={(1,1):command})
        runtime.run()
        self.assertTrue(runtime.refine_calls)
        for plane,arguments in runtime.refine_calls:
            self.assertIs(plane,runtime.tensor.planes[1])
            self.assertEqual(arguments['roi'],(170,20,80,80))
        observations = [p for cmd,p in runtime.decoded() if cmd == wire.OBSERVATION]
        self.assertTrue(observations)
        self.assertTrue(all(p[:4] == b'\0\7\0\3' and p[6] & wire.FLAG_REFINED for p in observations))

    def test_no_uart_ring_preview_can_select_station_number(self):
        runtime = Runtime(frames=2)
        runtime.run(bench=True,test_mode=4,test_pos=2)
        self.assertTrue(runtime.refine_calls)
        self.assertEqual(runtime.uart_args,[])

    def test_refine_api_failure_cannot_fall_back_in_precision_mode(self):
        runtime = Runtime(frames=2,commands={(1,1):wire.frame(wire.SET_TARGET,[1,2])+wire.frame(wire.SCAN,[4])})
        runtime.refine_failure = True
        runtime.run()
        self.assertFalse(any(cmd == wire.RING for cmd,_ in runtime.decoded()))
        self.assertTrue(runtime.sensor_stopped and runtime.uart_closed)

    def test_context_change_during_inference_discards_old_pose(self):
        runtime = Runtime(frames=2,commands={
            (1,1):wire.frame(wire.SET_CONTEXT,[0,10,0,1])+wire.frame(wire.SCAN,[1]),
            (1,2):wire.frame(wire.SET_CONTEXT,[0,11,0,2])})
        runtime.run()
        observations = [p for cmd,p in runtime.decoded() if cmd == wire.OBSERVATION]
        self.assertTrue(observations)
        self.assertTrue(all(p[:4] == bytes([0,11,0,2]) for p in observations))

    def test_ring_number_does_not_shift_when_another_ring_is_missing(self):
        rings = logic.detections(([[170,20,80,80]],[1],[0.9]),cfg.RING_IDS,rings=True)
        self.assertIsNone(logic.best(rings,1))
        self.assertEqual(logic.best(rings,2)[3:],(210,60))


if __name__ == '__main__':
    unittest.main()
