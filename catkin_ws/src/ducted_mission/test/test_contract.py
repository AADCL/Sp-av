import ast
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
BRINGUP = PACKAGE.parent / "ducted_bringup"


class PackageContractTests(unittest.TestCase):
    def test_source_first_spawn_can_import_generated_ros_subpackages(self):
        packages = (
            (PACKAGE / "src" / "ducted_mission" / "__init__.py",
             "ducted_mission", "msg",
             "MissionConfig = MissionCore = object\n"),
            (PACKAGE.parent / "ducted_navigation" / "src" /
             "ducted_navigation" / "__init__.py",
             "ducted_navigation", "srv",
             "Hull = Planner = PlannerConfig = Pose = Snapshot = "
             "Terrain = Vec3 = object\n"),
        )
        for initializer, package, generated_name, source_stub in packages:
            with self.subTest(package=package), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                source_package = root / "source" / package
                generated_package = root / "generated" / package
                source_package.mkdir(parents=True)
                (generated_package / generated_name).mkdir(parents=True)
                (source_package / "__init__.py").write_text(
                    initializer.read_text())
                stub_name = "mission.py" if package == "ducted_mission" else "planner.py"
                (source_package / stub_name).write_text(source_stub)
                (generated_package / "__init__.py").write_text("")
                (generated_package / generated_name / "__init__.py").write_text(
                    "GENERATED_VISIBLE = True\n")
                code = (
                    "import sys; sys.path[:0] = [{!r}, {!r}]; "
                    "from {}.{} import GENERATED_VISIBLE; "
                    "assert GENERATED_VISIBLE"
                ).format(str(root / "source"), str(root / "generated"),
                         package, generated_name)
                result = subprocess.run(
                    [sys.executable, "-c", code], text=True,
                    capture_output=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_package_generates_status_and_installs_node(self):
        cmake = (PACKAGE / "CMakeLists.txt").read_text()
        manifest = ElementTree.parse(PACKAGE / "package.xml").getroot()
        dependencies = {node.text for node in manifest.findall("depend")}
        self.assertIn("add_message_files(FILES MissionStatus.msg)", cmake)
        self.assertIn("generate_messages(DEPENDENCIES std_msgs)", cmake)
        self.assertLess(cmake.index("catkin_python_setup()"),
                        cmake.index("generate_messages"))
        self.assertIn("scripts/waypoint_mission.py", cmake)
        self.assertTrue({"ducted_msgs", "ducted_navigation", "nav_msgs",
                         "std_srvs", "tf2_geometry_msgs", "tf2_ros"}
                        .issubset(dependencies))

    def test_config_is_valid_and_does_not_contain_activation(self):
        config = yaml.safe_load((PACKAGE / "config" / "mission.yaml").read_text())
        self.assertGreaterEqual(len(config["waypoints"]), 2)
        self.assertNotIn("start", config)
        self.assertNotIn("enable_commands", config)

    def test_launch_is_disabled_and_maps_public_contract(self):
        root = ElementTree.parse(
            BRINGUP / "launch" / "waypoint_mission.launch").getroot()
        args = {node.attrib["name"]: node.attrib.get("default")
                for node in root.findall("arg")}
        self.assertEqual(args["enable_commands"], "false")
        self.assertIn("ducted_mission)/config/mission.yaml", args["mission_file"])
        node = root.find("node")
        self.assertEqual(node.attrib["ns"], "ducted/mission")
        params = {item.attrib["name"]: item.attrib["value"]
                  for item in node.findall("param")}
        self.assertEqual(params["enable_commands"], "$(arg enable_commands)")
        remaps = {item.attrib["from"]: item.attrib["to"]
                  for item in node.findall("remap")}
        self.assertEqual(remaps["navigation_command"], "/ducted/navigation/command")
        self.assertEqual(remaps["control_command"], "/ducted/control/command")
        self.assertEqual(remaps["status"], "/ducted/mission/status")

    def test_node_has_explicit_services_and_no_flight_transition_commands(self):
        script = (PACKAGE / "scripts" / "waypoint_mission.py").read_text()
        ast.parse(script)
        for service in ('"start"', '"pause"', '"resume"', '"cancel"'):
            self.assertIn("rospy.Service({}".format(service), script)
        self.assertIn("tf2_geometry_msgs.do_transform_pose", script)
        self.assertIn("rospy.Duration(0.0)", script)
        self.assertNotIn("rospy.Timer", script)
        self.assertIn("DeadlineProcess", script)
        for forbidden in ('"arm"', '"engage"', '"takeoff"', '"land"'):
            self.assertNotIn(forbidden, script)


if __name__ == "__main__":
    unittest.main()
