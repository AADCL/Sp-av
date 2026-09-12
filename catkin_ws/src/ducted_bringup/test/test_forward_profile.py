import importlib.util
import math
from pathlib import Path
import unittest


class ForwardProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[3] / 'prepare_forward_test.py'
        if not path.exists():
            cls.build = None
            return
        spec = importlib.util.spec_from_file_location('forward_preparation', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls.build = staticmethod(module.build_profile)
        factory=getattr(module,'build_contact_reference',None)
        cls.contact_factory=staticmethod(factory) if factory else None
        cls.relative=staticmethod(getattr(module,'build_relative_profile',lambda **kw:None))

    def test_px4_profile_uses_rise_not_absolute_one_or_measured_agl(self):
        p=self.relative(x=0.,y=0.,z=.4,yaw=math.pi/2)
        self.assertIsNotNone(p)
        self.assertAlmostEqual(p['waypoint']['position'][2],1.4)
        self.assertAlmostEqual(p['waypoint']['position'][1],3.)
        self.assertEqual(p['height_mode'],'takeoff_relative')
        self.assertEqual(p['finish'],'land')
        self.assertEqual(p['landing_mode'],'AUTO.LAND')
        self.assertNotIn('slow_land_speed',p)

    def profile(self, **updates):
        self.assertIsNotNone(self.build, 'task preparation script is missing')
        values = dict(x=0., y=0., z=2.1, yaw=0., agl=.1,
                      height=1., distance=3., hover=5.)
        values.update(updates)
        return self.build(**values)

    def test_nonzero_ground_origin_preserves_absolute_height(self):
        p = self.profile()
        self.assertAlmostEqual(p['takeoff_rise'], .9)
        self.assertEqual(p['waypoint']['position'], [3., 0., 3.])
        self.assertEqual(p['waypoint']['dwell'], 5.)

    def test_contact_reference_requires_explicit_confirmation_and_keeps_source(self):
        self.assertIsNotNone(self.contact_factory,'ground-contact preparation is missing')
        anchor=dict(x=0.,y=0.,z=2.1,yaw=0.)
        with self.assertRaises(ValueError):
            self.contact_factory(anchor,.1,'run-a',100.,False)
        r=self.contact_factory(anchor,.1,'run-a',100.,True)
        self.assertEqual(r['source'],'operator_ground_contact')
        self.assertAlmostEqual(r['ground_z'],2.)
        self.assertAlmostEqual(self.profile(agl=r['contact_agl'])['takeoff_rise'],.9)

    def test_forward_is_aircraft_heading_not_odom_x(self):
        p = self.profile(x=1., y=-1., yaw=math.pi/2)
        self.assertAlmostEqual(p['waypoint']['position'][0], 1.)
        self.assertAlmostEqual(p['waypoint']['position'][1], 2.)
        self.assertAlmostEqual(p['waypoint']['yaw'], math.pi/2)

    def test_ground_height_is_not_assumed_to_be_half_airframe_height(self):
        p = self.profile(z=.2, agl=.23)
        self.assertAlmostEqual(p['takeoff_rise'], .77)
        self.assertAlmostEqual(p['waypoint']['position'][2], .97)

    def test_outside_existing_flight_envelope_is_rejected(self):
        for values in ({'x':4.}, {'z':2.5}, {'y':4.,'yaw':math.pi/2}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.profile(**values)

    def test_nonfinite_or_unsafe_height_is_rejected(self):
        for values in ({'agl':float('nan')}, {'yaw':float('inf')}, {'agl':-.1},
                       {'agl':.9}, {'height':.5}, {'distance':0.}, {'hover':-1.}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.profile(**values)


if __name__ == '__main__':
    unittest.main()
