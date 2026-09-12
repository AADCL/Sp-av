import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src'))
from ducted_mission import ground_reference


class RelativeReferenceTests(unittest.TestCase):
    def make(self):
        self.assertTrue(hasattr(ground_reference,'RelativeReference'))
        return ground_reference.RelativeReference(dict(confirmed=True,run_id='r',frame_id='odom',
            prepared_stamp=100.,anchor=dict(x=0.,y=0.,z=.4,yaw=0.)), 'r')

    def test_initialization_and_airborne_height_need_no_terrain(self):
        r=self.make()
        self.assertEqual(r.check(0,0,.4,0,0,0,True,101), '')
        r.begin(101)
        self.assertEqual(r.check(3,0,1.4,0,.02,.1,False,120), '')
        with self.assertRaises(ValueError):r.begin(121)

    def test_motion_expiry_and_clock_reversal_reject(self):
        self.assertIn('moved',self.make().check(.1,0,.4,0,0,0,True,101))
        self.assertIn('expired',self.make().check(0,0,.4,0,0,0,True,401))
        self.assertIn('clock',self.make().check(0,0,.4,0,0,0,True,99))


if __name__=='__main__':unittest.main()
