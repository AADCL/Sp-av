import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from ducted_control.rc import RCDecoder


def config():
    return {'channels': {'roll': 1, 'pitch': 2, 'throttle': 3, 'yaw': 4,
                         'mode': 5, 'land': 6, 'arm': 7, 'kill': 8, 'trigger': 12},
            'calibration': {str(i): {'min': 1044 if i < 5 else 1000,
                                     'max': 1944 if i < 5 else 2000,
                                     'trim': 1494 if i < 5 else 1500,
                                     'reverse': -1 if i == 2 else 1}
                            for i in (1, 2, 3, 4, 5, 6, 7, 8, 12)},
            'timeout': 0.5, 'future_tolerance': 0.05, 'deadzone': 0.05,
            'hysteresis': 0.03, 'pwm_margin': 75}


class RCDecoderTest(unittest.TestCase):
    def setUp(self):
        self.decoder = RCDecoder(config())
        self.raw = [1494] * 4 + [1044] * 8

    def feed(self, raw=None, stamp=100., wall=10., ros_now=None, rssi=255):
        return self.decoder.update(self.raw if raw is None else raw, stamp,
                                   stamp if ros_now is None else ros_now, wall, rssi)

    def test_real_endpoints_trim_and_pitch_reverse(self):
        self.raw[:4] = [1944, 1944, 1044, 1494]
        r = self.feed()
        self.assertTrue(r.valid)
        self.assertEqual((r.roll, r.pitch, r.throttle, r.yaw), (1., -1., 0., 0.))
        self.raw[2] = 1494
        self.assertAlmostEqual(self.feed(stamp=100.1).throttle, .5)

    def test_kill_is_level_detected_on_first_frame(self):
        self.raw[7] = 1944
        r = self.feed()
        self.assertTrue(r.valid)
        self.assertTrue(r.kill_switch)

    def test_short_frame_and_bad_pwm_clear_previous_state(self):
        self.assertTrue(self.feed().valid)
        self.assertFalse(self.feed(raw=[1500] * 4, stamp=100.1).valid)
        self.assertFalse(self.decoder.evaluate(100.1, 10.1).valid)
        self.raw[0] = 65535
        self.assertFalse(self.feed(stamp=100.2).valid)

    def test_duplicate_backward_old_and_future_stamps_rejected(self):
        self.assertTrue(self.feed().valid)
        self.assertFalse(self.feed().valid)
        self.assertFalse(self.feed(stamp=99.9).valid)
        self.assertFalse(self.feed(stamp=100.2, ros_now=101.).valid)
        self.assertFalse(self.feed(stamp=102., ros_now=100.).valid)
        self.assertFalse(self.feed(stamp=0.).valid)

    def test_frozen_ros_clock_does_not_keep_rc_valid(self):
        self.feed()
        self.assertFalse(self.decoder.evaluate(100., 10.51).valid)

    def test_malformed_newer_frame_prevents_old_switch_replay(self):
        self.feed()
        malformed = list(self.raw); malformed[0] = 65535
        self.assertFalse(self.feed(raw=malformed, stamp=100.4, wall=10.4).valid)
        self.assertFalse(self.feed(stamp=100.2, ros_now=100.4, wall=10.4).valid)
        self.assertFalse(self.feed(stamp=100.4, wall=10.4).valid)
        self.assertTrue(self.feed(stamp=100.5, wall=10.5).valid)

    def test_unknown_rssi_allowed_but_zero_rssi_rejected(self):
        self.assertTrue(self.feed(rssi=255).valid)
        self.assertFalse(self.feed(stamp=100.1, rssi=0).valid)

    def test_mode_hysteresis_prevents_boundary_chatter(self):
        self.assertEqual(self.feed().mode, 'manual')
        self.raw[4] = 1260
        self.assertEqual(self.feed(stamp=100.1).mode, 'manual')
        self.raw[4] = 1500
        self.assertEqual(self.feed(stamp=100.2).mode, 'hold')
        self.raw[4] = 1740
        self.assertEqual(self.feed(stamp=100.3).mode, 'hold')
        self.raw[4] = 1944
        self.assertEqual(self.feed(stamp=100.4).mode, 'command')

    def test_invalid_sticks_do_not_preserve_active_switches(self):
        self.raw[4] = 1944
        self.raw[5] = 1944
        self.assertTrue(self.feed().land_switch)
        self.raw[0] = math.nan
        r = self.feed(stamp=100.1)
        self.assertFalse(r.valid)
        self.assertTrue(r.kill_switch)
        self.assertEqual(r.mode, 'manual')

    def test_missing_noninteger_and_colliding_channels_rejected(self):
        for bad in (0, 1.5, True, 1):
            c = config(); c['channels']['mode'] = bad
            with self.assertRaises(ValueError): RCDecoder(c)

    def test_invalid_calibration_rejected(self):
        for field, value in [('max', 1000), ('trim', math.nan), ('reverse', 0)]:
            c = config(); c['calibration']['1'][field] = value
            with self.assertRaises(ValueError): RCDecoder(c)

    def test_invalid_time_configuration_rejected(self):
        for field, value in [('timeout', 0), ('deadzone', 1), ('hysteresis', -.1),
                             ('future_tolerance', math.nan), ('pwm_margin', -1)]:
            c = config(); c[field] = value
            with self.assertRaises(ValueError): RCDecoder(c)


if __name__ == '__main__': unittest.main()
