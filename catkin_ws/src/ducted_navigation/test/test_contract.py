"""Static safety and integration contract for the navigation package."""
import pathlib
import unittest
import xml.etree.ElementTree as ET


PKG = pathlib.Path(__file__).resolve().parents[1]
SOURCE_ROOT = PKG.parent


class NavigationContractTest(unittest.TestCase):
    def test_public_interface_is_stable(self):
        service = (PKG / "srv/Navigate.srv").read_text(encoding="utf-8")
        status = (PKG / "msg/NavigationStatus.msg").read_text(encoding="utf-8")
        self.assertEqual(service, "string command\nstring request_id\ngeometry_msgs/PoseStamped target\n---\nbool accepted\nstring message\n")
        for field in ("request_id", "state", "reason", "odom_age", "cloud_age",
                      "terrain_age", "controller_age", "clearance", "progress"):
            self.assertIn(field, status)

    def test_defaults_inhibit_output_and_require_measured_geometry(self):
        config = (PKG / "config/local_avoidance.yaml").read_text(encoding="utf-8")
        launch = (SOURCE_ROOT / "ducted_bringup/launch/local_avoidance.launch").read_text(
            encoding="utf-8")
        self.assertIn("enable_output: false", config)
        self.assertIn("geometry_confirmed: false", config)
        self.assertIn('name="geometry_file" default="$(find ducted_bringup)/config/airframe_geometry.yaml"', launch)
        # Frozen launch retains compatibility arguments but does not load runtime settings.
        self.assertFalse(ET.fromstring(launch).findall(".//rosparam"))
        self.assertIn('name="enable_output" default="false"', launch)
        self.assertIn('name="geometry_confirmed" default="false"', launch)
        self.assertIn('name="config_file" default="$(find ducted_navigation)/config/local_avoidance.yaml"', launch)

    def test_navigation_has_no_flight_or_mission_authority(self):
        roots = (PKG / "scripts", PKG / "src", PKG / "CMakeLists.txt", PKG / "package.xml")
        files = []
        for root in roots:
            files.extend(root.rglob("*.py") if root.is_dir() else (root,))
        text = "\n".join(path.read_text(encoding="utf-8", errors="ignore") for path in files)
        for forbidden in ("mavros_msgs", "CommandBool", "SetMode", "arming", '"takeoff"', "'takeoff'",
                          "AUTO.LAND", "RCState"):
            self.assertNotIn(forbidden, text)
        self.assertIn('rospy.Publisher("target", FlightSetpoint', text)

    def test_frozen_launch_cannot_start_navigation_or_control(self):
        for name in ("local_avoidance", "ego_planner", "ego_offboard"):
            root = ET.parse(SOURCE_ROOT / ("ducted_bringup/launch/" + name + ".launch")).getroot()
            nodes = root.findall(".//node")
            self.assertEqual(len(nodes), 1)
            self.assertEqual(nodes[0].attrib["type"], "frozen_ego.py")
            self.assertEqual(nodes[0].attrib["required"], "true")
            self.assertFalse(root.findall(".//include"))
            self.assertFalse(root.findall(".//remap"))


if __name__ == "__main__":
    unittest.main()
