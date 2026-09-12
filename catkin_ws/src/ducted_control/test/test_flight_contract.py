from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[3]
CONTROL = ROOT / 'src/ducted_control'
MSGS = ROOT / 'src/ducted_msgs'
BRINGUP = ROOT / 'src/ducted_bringup'


class FlightContractTest(unittest.TestCase):
    def test_messages_and_service_preserve_absolute_pose_contract(self):
        self.assertEqual((MSGS / 'msg/FlightSetpoint.msg').read_text().splitlines(), [
            'std_msgs/Header header', 'string request_id', 'geometry_msgs/Pose pose'])
        self.assertEqual((MSGS / 'msg/FlightControlStatus.msg').read_text().splitlines(), [
            'std_msgs/Header header', 'string state', 'string reason', 'bool ready',
            'string request_id', 'geometry_msgs/Pose target',
            'string height_source', 'bool height_reference_valid'])
        self.assertEqual((MSGS / 'srv/FlightCommand.srv').read_text().splitlines(), [
            'string command', 'geometry_msgs/PoseStamped target', '---',
            'bool accepted', 'string message'])

    def test_shipped_configuration_is_disabled_and_base_timeout_matches_2hz_source(self):
        flight = yaml.safe_load((CONTROL / 'config/flight.yaml').read_text())['flight']
        self.assertIs(flight['enable_flight_output'], False)
        self.assertGreaterEqual(flight['base_ready_timeout'], 1.5)
        self.assertLessEqual(flight['external_ready_timeout'], .5)
        self.assertEqual(flight['frame_id'], 'odom')
        self.assertGreaterEqual(flight['loop_hz'], 20.)

    def test_launch_is_independent_and_exposes_isolated_backend_remaps(self):
        launch = (BRINGUP / 'launch/flight_control.launch').read_text()
        for required in ('enable_flight_output" default="false',
                         'mapping_confirmed" default="false',
                         '/ducted/control/target', '/ducted/control/command',
                         '/ducted/control/status', '/ducted/control/ready',
                         'setpoint_topic', 'set_mode_service', 'fcu_state_topic',
                         'local_pose_topic', 'external_pose_topic', 'extended_state_topic'):
            self.assertIn(required, launch)
        for forbidden in ('base_system.launch', 'mavros.launch',
                          'external_odometry.launch', 'terrain_height.launch'):
            self.assertNotIn(forbidden, launch)

    def test_build_metadata_generates_and_installs_flight_interfaces(self):
        msg_cmake = (MSGS / 'CMakeLists.txt').read_text()
        for item in ('geometry_msgs', 'FlightSetpoint.msg', 'FlightControlStatus.msg',
                     'FlightCommand.srv', 'add_service_files'):
            self.assertIn(item, msg_cmake)
        control_cmake = (CONTROL / 'CMakeLists.txt').read_text()
        for item in ('geometry_msgs', 'nav_msgs', 'scripts/flight_controller.py',
                     'test/test_flight.py', 'test/test_flight_runtime.py'):
            self.assertIn(item, control_cmake)


if __name__ == '__main__':
    unittest.main()
