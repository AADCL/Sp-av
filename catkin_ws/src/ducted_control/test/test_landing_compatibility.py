import pathlib, sys, unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src'))
from ducted_control.landing import landing_compatibility

class LandingCompatibilityTests(unittest.TestCase):
    def test_current_px4_parameters_cannot_confirm_bounded_slow_landing(self):
        ok,reason=landing_compatibility(17564416,.7,1.,.2)
        self.assertFalse(ok)
        self.assertIn('0.63',reason)

    def test_unknown_firmware_is_not_certified_by_speed_alone(self):
        self.assertFalse(landing_compatibility(0,.2,1.,.2)[0])

    def test_missing_parameters_are_rejected(self):
        self.assertFalse(landing_compatibility(17564416,float('nan'),1.,.2)[0])

if __name__=='__main__':unittest.main()
