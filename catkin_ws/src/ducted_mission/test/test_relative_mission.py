import unittest
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_mission import MissionCore, MissionConfig, config_dict, gates

class RelativeMissionTests(unittest.TestCase):
    def test_relative_mode_requires_other_gates_but_not_terrain(self):
        c=MissionCore(MissionConfig.from_dict(config_dict()),True)
        self.assertEqual(c._gate_reason(gates(height_mode='takeoff_relative',terrain_valid=False,
                                             terrain_ready=False,terrain_age=float('inf'))),'')
        self.assertIn('RC',c._gate_reason(gates(height_mode='takeoff_relative',rc_valid=False)))
        self.assertIn('terrain',c._gate_reason(gates(terrain_valid=False)))

if __name__=='__main__':unittest.main()
