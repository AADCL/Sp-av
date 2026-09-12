import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))


class GroundReferenceTests(unittest.TestCase):
    def reference(self, **changes):
        spec = importlib.util.find_spec('ducted_mission.ground_reference')
        self.assertIsNotNone(spec, 'explicit contact reference is not implemented')
        from ducted_mission.ground_reference import TakeoffReference
        data = dict(confirmed=True, source='operator_ground_contact', frame_id='odom',
                    run_id='run-a', prepared_stamp=100., contact_agl=.1, ground_z=2.,
                    anchor=dict(x=0., y=0., z=2.1, yaw=0.))
        data.update(changes)
        return TakeoffReference(data, 'run-a')

    def read(self, ref, **changes):
        values = dict(x=0., y=0., z=2.1, yaw=0., tilt=0., speed=0.,
                      on_ground=True, ros_now=101., wall_now=10., measured=None)
        values.update(changes)
        return ref.evaluate(**values)

    def test_ground_contact_allows_preparation_without_lidar_measurement(self):
        r = self.read(self.reference())
        self.assertTrue(r.valid, r.reason)
        self.assertEqual(r.source, 'CONTACT_REFERENCE')
        self.assertFalse(r.measured)
        self.assertAlmostEqual(r.agl, .1)

    def test_confirmation_session_and_geometry_must_be_explicit(self):
        for change in ({'confirmed':False}, {'run_id':'another-run'},
                       {'contact_agl':0.}, {'ground_z':1.}, {'source':'ON_GROUND'}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.reference(**change)

    def test_movement_or_expired_preparation_requires_new_confirmation(self):
        for change in ({'z':2.3}, {'yaw':.3}, {'tilt':.3}, {'speed':.2},
                       {'on_ground':False}, {'ros_now':401.}):
            with self.subTest(change=change):
                self.assertFalse(self.read(self.reference(), **change).valid)

    def test_blind_climb_is_bounded_in_height_time_and_horizontal_motion(self):
        ref=self.reference();ref.begin(101.,10.)
        self.assertTrue(self.read(ref,z=2.6,on_ground=False,ros_now=102.,wall_now=11.).valid)
        for change in ({'z':3.2}, {'x':.3}, {'wall_now':19.,'ros_now':110.}):
            other=self.reference();other.begin(101.,10.)
            self.assertFalse(self.read(other,on_ground=False,**change).valid)

    def test_handover_needs_distinct_consistent_measurements_and_never_falls_back(self):
        ref=self.reference();ref.begin(101.,10.)
        for i in range(3):
            r=self.read(ref,z=2.6,on_ground=False,ros_now=102.+i*.1,wall_now=11.+i*.1,
                        measured=dict(ground_z=2.,agl=.6,stamp=102.+i*.1))
            self.assertEqual(r.measured, i==2)
        self.assertEqual(r.source,'LIDAR')
        self.assertFalse(self.read(ref,z=2.6,on_ground=False,ros_now=102.4,wall_now=11.4).valid)

    def test_conflicting_measured_floor_does_not_silently_replace_contact_floor(self):
        ref=self.reference();ref.begin(101.,10.)
        self.assertFalse(self.read(ref,z=2.6,on_ground=False,
            measured=dict(ground_z=2.3,agl=.3,stamp=101.)).valid)

    def test_high_rate_measurements_accumulate_the_required_time_span(self):
        ref=self.reference();ref.begin(101.,10.)
        for i in range(12):
            r=self.read(ref,z=2.6,on_ground=False,ros_now=102.+i*.02,wall_now=11.+i*.02,
                        measured=dict(ground_z=2.,agl=.6,stamp=102.+i*.02))
        self.assertTrue(r.measured)

    def test_clock_rollback_revokes_contact_reference(self):
        ref=self.reference();ref.begin(101.,10.)
        self.assertTrue(self.read(ref,z=2.5,on_ground=False,ros_now=102.,wall_now=11.).valid)
        self.assertFalse(self.read(ref,z=2.5,on_ground=False,ros_now=101.,wall_now=12.).valid)


if __name__=='__main__': unittest.main()
