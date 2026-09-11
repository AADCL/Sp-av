#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


PKG = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PKG.parent


class MappingLaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch_path = PKG / "launch" / "mapping.launch"
        cls.config_path = PKG / "config" / "mapping_mid360.yaml"
        cls.launch_text = cls.launch_path.read_text(encoding="utf-8")
        cls.root = ET.fromstring(cls.launch_text)
        cls.config = yaml.safe_load(cls.config_path.read_text(encoding="utf-8"))

    def test_mapping_is_a_separate_optional_launch(self):
        self.assertTrue((PKG / "launch" / "base_system.launch").exists())
        self.assertNotIn("mapping.launch", (PKG / "launch" / "base_system.launch").read_text())

    def test_fast_lio_is_required_and_waits_for_base_readiness(self):
        node = self.root.find(".//node[@pkg='fast_lio_sam'][@type='fastlio_sam_mapping']")
        self.assertIsNotNone(node)
        self.assertEqual(node.attrib.get("required"), "true")
        params = {p.attrib["name"]: p.attrib.get("value") for p in node.findall("param")}
        args = {a.attrib["name"]: a.attrib.get("default") for a in self.root.findall("arg")}
        self.assertEqual(params.get("require_base_ready"), "true")
        self.assertEqual(args.get("base_ready_topic"), "/ducted/system/ready")
        self.assertEqual(params.get("base_ready_topic"), "$(arg base_ready_topic)")

    def test_public_topics_have_stable_ducted_names(self):
        required = {
            "Odometry": "/ducted/localization/odom",
            "path": "/ducted/localization/path",
            "cloud_registered": "/ducted/mapping/cloud_registered",
            "fast_lio_sam/mapping/map_global_optimized": "/ducted/mapping/map_global",
        }
        remaps = {r.attrib["from"].lstrip("/"): r.attrib["to"] for r in self.root.findall(".//remap")}
        for source, target in required.items():
            self.assertEqual(remaps.get(source), target)

    def test_fixed_map_frame_keeps_absolute_z(self):
        self.assertEqual(self.config["frames"]["map"], "camera_init")
        self.assertEqual(self.config["frames"]["body"], "body")
        self.assertFalse(self.config["height"]["subtract_ground_height"])

    def test_mid360_internal_extrinsic_uses_local_reference(self):
        self.assertEqual(
            self.config["mapping"]["extrinsic_T"],
            [-0.011, -0.02329, 0.04412],
        )
        self.assertEqual(
            self.config["mapping"]["extrinsic_R"],
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )

    def test_no_old_vehicle_or_ground_mode_nodes(self):
        self.assertNotIn("0.4567", self.launch_text)
        self.assertNotIn("map_remove_head", self.launch_text)
        self.assertNotIn("fix_odom_sam", self.launch_text)
        self.assertNotIn("trans_imu", self.launch_text)

    def test_map_is_saved_only_by_explicit_service_call(self):
        self.assertFalse(self.config["pcd_save"]["pcd_save_en"])
        self.assertFalse(self.config["export"]["save_on_shutdown"])
        self.assertEqual(self.config["export"]["directory"], "/home/nrc/catkin_ws/maps")


class FastLioPortContractTest(unittest.TestCase):
    def test_driver2_and_frame_parameters_are_used(self):
        package = WORKSPACE_SRC / "fast_lio_sam"
        cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")
        manifest = (package / "package.xml").read_text(encoding="utf-8")
        source = (package / "src" / "laserMapping.cpp").read_text(encoding="utf-8")
        preprocess = (package / "src" / "preprocess.h").read_text(encoding="utf-8")
        self.assertIn("livox_ros_driver2", cmake)
        self.assertIn("livox_ros_driver2", manifest)
        self.assertIn("livox_ros_driver2/CustomMsg.h", preprocess)
        self.assertIn('nh.param<std::string>("frames/map"', source)
        self.assertIn('nh.param<std::string>("frames/body"', source)
        self.assertIn('nh.param<bool>("require_base_ready"', source)
        self.assertIn("p_pre->lidar_type == AVIA || p_pre->lidar_type == MID360", source)


if __name__ == "__main__":
    unittest.main()
