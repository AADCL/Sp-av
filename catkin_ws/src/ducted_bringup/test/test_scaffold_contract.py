#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_DIR.parent


class ScaffoldContractTest(unittest.TestCase):
    def test_base_system_starts_required_hardware_and_monitor(self):
        root = ET.parse(PACKAGE_DIR / "launch" / "base_system.launch").getroot()
        args = {element.attrib["name"]: element.attrib.get("default")
                for element in root.findall("arg")}

        self.assertEqual(args["start_mavros"], "true")
        self.assertEqual(args["start_livox"], "true")
        self.assertEqual(args["start_health_monitor"], "true")
        self.assertEqual(args["fcu_url"], "/dev/ttyTHS0:921600")

        include_files = [element.attrib["file"] for element in root.findall("include")]
        self.assertIn("$(find ducted_bringup)/launch/mavros.launch", include_files)
        self.assertIn("$(find ducted_bringup)/launch/livox.launch", include_files)

        monitor = root.find("./group/node[@type='base_health_monitor.py']")
        self.assertIsNotNone(monitor)
        self.assertEqual(monitor.attrib["required"], "true")

    def test_height_semantics_are_separate(self):
        with (PACKAGE_DIR / "config" / "base_system.yaml").open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        self.assertEqual(config["height"]["localization_reference"], "absolute_map_height")
        self.assertEqual(config["height"]["avoidance_reference"], "above_ground_level")
        self.assertNotEqual(
            config["topics"]["terrain_height"],
            config["topics"]["mavros_state"],
        )

    def test_health_monitor_accepts_the_configured_livox_wire_format(self):
        monitor_text = (PACKAGE_DIR / "scripts" / "base_health_monitor.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("rospy.AnyMsg", monitor_text)
        self.assertNotIn("PointCloud2", monitor_text)

    def test_terrain_height_message_contract(self):
        fields = [
            line.strip()
            for line in (WORKSPACE_SRC / "ducted_msgs" / "msg" / "TerrainHeight.msg")
            .read_text(encoding="utf-8")
            .splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]

        self.assertEqual(fields, [
            "std_msgs/Header header",
            "float64 ground_z",
            "float64 agl",
            "float64 variance",
            "bool valid",
        ])

    def test_excluded_platform_and_ground_mode_are_absent(self):
        launch_text = (PACKAGE_DIR / "launch" / "base_system.launch").read_text(encoding="utf-8").lower()
        self.assertNotIn("ccs", launch_text)
        self.assertNotIn("ground_mode", launch_text)


if __name__ == "__main__":
    unittest.main()
