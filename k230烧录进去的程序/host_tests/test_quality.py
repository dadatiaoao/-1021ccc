from pathlib import Path
import sys
import unittest
import math
from types import SimpleNamespace
from unittest.mock import patch

sys.dont_write_bytecode = True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'transport_vision'))
import vision_config as cfg
import vision_protocol as wire
from vision_logic import Controller, detections
from vision_quality import Stability
import vision_quality
from vision_tasks import parse_task, expand_task
from vision_settings import validate
from ring_refinement import family_center


def item(ident=1,cx=100,cy=100,size=40):
    return (ident,0.9,(cx-size/2,cy-size/2,size,size),cx,cy)


def decoded(state):
    return wire.Parser().feed(b''.join(state.report()))


class QualityTests(unittest.TestCase):
    def test_canmv_math_without_hypot_supports_tracking_and_circle_fit(self):
        with patch.object(vision_quality, 'math', SimpleNamespace(sqrt=math.sqrt)):
            quality = Stability()
            quality.update(item(), 0, 0)
            quality.update(item(cx=103, cy=104), 1, 100)
            self.assertAlmostEqual(quality.speed, 50.0)
            quality.reset()
            for i in range(4):
                quality.update(item(), i, i*100)
            self.assertTrue(quality.stable)
            center, reason = family_center([(70,70,r) for r in (10,20,30)], (20,20,100,100))
            self.assertEqual(reason, 'CIRCLE_FAMILY')
            self.assertEqual(center[:2], (70,70))

    def test_static_requires_distinct_frames_and_elapsed_time(self):
        quality = Stability()
        for index in range(3):
            self.assertFalse(quality.update(item(),index,index*100))
        for _ in range(20):
            self.assertFalse(quality.update(item(),2,250))
        self.assertTrue(quality.update(item(),3,300))

    def test_motion_and_scale_change_break_stability(self):
        for changed in (item(cx=115),item(size=50)):
            quality = Stability()
            for i in range(4):quality.update(item(),i,100*i)
            self.assertTrue(quality.stable)
            self.assertFalse(quality.update(changed,4,400))

    def test_loss_gap_and_backward_frames_reset_window(self):
        for frame,at,target in ((4,800,item()),(4,400,None),(1,400,item())):
            quality = Stability()
            for i in range(4):quality.update(item(),i,100*i)
            self.assertFalse(quality.update(target,frame,at))

    def test_frame_sequence_wrap(self):
        quality = Stability()
        for i in range(4):quality.update(item(),(65534+i)&65535,100*i)
        self.assertTrue(quality.stable)

    def test_nan_infinite_fractional_class_and_misaligned_results_rejected(self):
        self.assertEqual(detections(([[10,10,20,20]],[0],[]),cfg.COLOR_IDS),[])
        for cls,box,score in ((0,[float('nan'),1,2,3],.9),(float('inf'),[1,1,2,3],.9),
                              (1.5,[1,1,2,3],.9),(0,[1,1,2,3],float('nan'))):
            self.assertEqual(detections(([box],[cls],[score]),cfg.COLOR_IDS),[])

    def test_multiple_same_colour_rejected_instead_of_switching(self):
        state = Controller();state.control(wire.SCAN,[1])
        state.observe([item(),item(cx=200)],[],1,0)
        self.assertEqual(state.reason,'AMBIGUOUS')
        self.assertEqual(decoded(state),[(wire.NOTFOUND,b'')])

    def test_roi_resolves_duplicate_targets(self):
        state = Controller();state.control(wire.SCAN,[1])
        with patch.object(cfg,'MATERIAL_ROI',(50,50,100,100)):
            state.observe([item(),item(cx=200)],[],1,0)
        self.assertEqual(decoded(state)[0][0],wire.FOUND)

    def test_stale_observation_revoked_with_context(self):
        state = Controller();state.control(wire.SET_CONTEXT,[0,1,0,2]);state.control(wire.SCAN,[1])
        state.observe([item()],[],1,100,age_ms=301)
        result = decoded(state)
        self.assertEqual(result[0],(wire.NOTFOUND,b''))
        self.assertEqual(result[-1][1][6],wire.FLAG_STALE)
        self.assertEqual(result[-1][1][-4:],b'\0\0\0\0')

    def test_pick_candidate_does_not_authorize_uncalibrated_grab(self):
        state = Controller();state.control(wire.SCAN,[3])
        for i in range(4):state.observe([item()],[],i,100*i)
        self.assertTrue(state.observation[2] & wire.FLAG_STABLE)
        self.assertFalse(state.observation[2] & wire.FLAG_PICK_READY)

    def test_calibrated_pick_requires_roi_and_stable_new_frames(self):
        state = Controller();state.control(wire.SCAN,[3])
        with patch.object(cfg,'PICK_CALIBRATED',True),patch.object(cfg,'PICK_ROI',(70,70,60,60)):
            for i in range(4):state.observe([item()],[],i,100*i)
            self.assertTrue(state.observation[2] & wire.FLAG_PICK_READY)
            state.observe([],[],4,400)
            self.assertIsNone(state.observation)

    def test_repeated_reports_do_not_increment_place_stability(self):
        state = Controller();state.control(wire.SCAN,[2])
        state.observe([item()], [item(size=100)], 1,0, refinement=(100,100,3))
        for _ in range(20):decoded(state)
        self.assertEqual(state.place_hits,0)

    def test_place_candidate_requires_all_calibration_and_stability_conditions(self):
        state = Controller();state.control(wire.SCAN,[2])
        with patch.object(cfg,'PLACE_CHECK_CALIBRATED',True),patch.object(cfg,'RING_REFINEMENT_CALIBRATED',True):
            for i in range(7):state.observe([item()], [item(size=100)],i,100*i,refinement=(100,100,3))
            self.assertEqual(decoded(state)[1],(wire.PLACE_CHECK,b'\1\1\1'))
            state.observe([],[],7,700)
            self.assertEqual(decoded(state)[1],(wire.PLACE_CHECK,b'\1\1\0'))

    def test_partial_box_never_becomes_pick_ready(self):
        state = Controller();state.control(wire.SCAN,[3])
        with patch.object(cfg,'PICK_CALIBRATED',True),patch.object(cfg,'PICK_ROI',(0,0,100,100)):
            for i in range(5):state.observe([item(cx=5,cy=50)],[],i,100*i)
            self.assertFalse(state.observation[2] & wire.FLAG_PICK_READY)

    def test_context_echo_and_pose_change_revoke_previous_observation(self):
        state = Controller();state.control(wire.SET_CONTEXT,[0x12,0x34,0,2]);state.control(wire.SCAN,[4])
        state.observe([], [item(size=100)], 7,0, refinement=(101,102,3))
        result = decoded(state)
        self.assertEqual(result[0],(wire.RING,bytes([1])+wire.u16(202)+wire.u16(204)))
        self.assertEqual(result[-1][0],wire.OBSERVATION)
        self.assertEqual(result[-1][1][:6],b'\x12\x34\0\2\0\7')
        self.assertTrue(result[-1][1][6] & wire.FLAG_REFINED)
        self.assertFalse(result[-1][1][6] & wire.FLAG_CALIBRATED)
        state.control(wire.SET_CONTEXT,[0x12,0x35,0,3])
        self.assertIsNone(state.observation)
        self.assertEqual(decoded(state)[-1][1][:4],b'\x12\x35\0\3')

    def test_precision_mode_never_falls_back_to_box_center(self):
        state = Controller();state.control(wire.SCAN,[4])
        state.observe([], [item(size=100)],1,0)
        self.assertEqual(decoded(state),[(wire.NOTFOUND,b'')])

    def test_legacy_mode_keeps_coarse_center_and_no_extended_packet(self):
        state = Controller();state.control(wire.SCAN,[2])
        state.observe([], [item(size=100)],1,0,refinement=(105,106,3))
        result = decoded(state)
        self.assertEqual(result[0][1],bytes([1])+wire.u16(200)+wire.u16(200))
        self.assertEqual(result[1],(wire.PLACE_CHECK,b'\1\1\0'))
        self.assertFalse(any(cmd == wire.OBSERVATION for cmd,_ in result))

    def test_qr_requires_consecutive_consistent_new_frames(self):
        state = Controller();state.control(wire.SCAN,[0])
        good = '156+123+516+231'
        self.assertEqual(state.qr_report([good],1),[])
        self.assertEqual(state.qr_report([good],1),[])
        self.assertEqual(state.qr_report([],2),[])
        self.assertEqual(state.qr_report([good],3),[])
        self.assertEqual(state.qr_report([good],4),[])
        self.assertTrue(state.qr_report([good],5))
        self.assertEqual(state.qr_report([good],6),[])
        self.assertEqual(state.task_text,good)

    def test_two_different_valid_qrs_are_ambiguous(self):
        state = Controller();state.control(wire.SCAN,[0])
        for i in range(5):self.assertEqual(state.qr_report(['156+123+516+231','123+123+321+123'],i),[])

    def test_second_batch_storage_is_mapped_by_colour(self):
        plan = expand_task('156+123+516+231')
        self.assertEqual([row['storage_slot'] for row in plan[3:]],[2,1,3])
        self.assertEqual([row['rough_slot'] for row in plan[3:]],[2,3,1])
        self.assertTrue(all(row['layer'] == 2 for row in plan[3:]))

    def test_invalid_task_format_rejected(self):
        for text in ('111+666+222+555','12+3123+321+123','1 56+123+516+231',None,b'\xff'):
            self.assertIsNone(parse_task(text))

    def test_no_context_or_invalid_command_does_not_change_state(self):
        state = Controller();state.control(wire.SCAN,[0]);version = state.version
        state.qr_report(['156+123+516+231'],1)
        state.control(wire.SET_COLOR,[0])
        self.assertEqual(state.version,version)
        self.assertEqual(state.qr_hits,1)

    def test_parser_fragment_noise_checksum_and_max_frame(self):
        packet = wire.frame(wire.OBSERVATION,bytes(range(12)))
        self.assertEqual(len(packet),16)
        parser = wire.Parser()
        self.assertEqual(parser.feed(b'noise'+packet[:3]),[])
        self.assertEqual(parser.feed(packet[3:]),[(wire.OBSERVATION,bytes(range(12)))])
        damaged = packet[:-1]+bytes([packet[-1]^255])
        self.assertEqual(wire.Parser().feed(damaged+packet),[(wire.OBSERVATION,bytes(range(12)))])

    def test_invalid_settings_are_rejected(self):
        self.assertTrue(validate())
        for name,value in (('CONFIDENCE',float('nan')),('PICK_CALIBRATED',True),
                           ('PICK_ROI',(600,0,80,80)),('STABLE_MIN_FRAMES',1),('COLOR_IDS',(1,)*6)):
            with patch.object(cfg,name,value):
                with self.assertRaises(ValueError):validate()


class CircleFamilyTests(unittest.TestCase):
    def test_concentric_family_recovers_center_with_box_bias(self):
        center,reason = family_center([(72,69,10),(72,70,20),(73,69,30),(50,50,4)],(20,20,100,100))
        self.assertEqual(reason,'CIRCLE_FAMILY')
        self.assertAlmostEqual(center[0],72+1/3)
        self.assertNotAlmostEqual(center[0],70)

    def test_one_circle_or_line_edges_do_not_establish_scoring_ring_identity(self):
        for circles in ([(70,70,20)],[(70,70,20),(70,70,21),(70,70,22)]):
            self.assertIsNone(family_center(circles,(20,20,100,100))[0])

    def test_oblique_or_two_families_rejected(self):
        self.assertEqual(family_center([],(10,10,100,50))[1],'OBLIQUE_VIEW')
        circles=[(cx,70,r) for cx in (55,85) for r in (5,8,11)]
        self.assertEqual(family_center(circles,(20,20,100,100))[1],'AMBIGUOUS_FAMILIES')

    def test_outside_and_nonfinite_circles_rejected(self):
        self.assertIsNone(family_center([(70,70,60),(float('nan'),70,10)],(20,20,100,100))[0])


if __name__ == '__main__':unittest.main()
