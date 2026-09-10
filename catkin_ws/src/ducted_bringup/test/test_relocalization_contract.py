#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


PKG = pathlib.Path(__file__).resolve().parents[1]
SRC = PKG.parent


class RelocalizationContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch_path = PKG / "launch" / "relocalization.launch"
        cls.config_path = PKG / "config" / "relocalization_mid360.yaml"
        cls.launch_text = cls.launch_path.read_text(encoding="utf-8")
        cls.root = ET.fromstring(cls.launch_text)
        cls.config = yaml.safe_load(cls.config_path.read_text(encoding="utf-8"))

    def test_relocalization_is_optional_and_separate(self):
        for name in ("base_system.launch", "mapping.launch"):
            self.assertNotIn("relocalization.launch", (PKG / "launch" / name).read_text())

    def test_required_node_waits_for_base_and_uses_explicit_map(self):
        node = self.root.find(
            ".//node[@pkg='sfast_lio'][@type='sfastlio_mapping_relocalization']"
        )
        self.assertIsNotNone(node)
        self.assertEqual(node.attrib.get("required"), "true")
        params = {p.attrib["name"]: p.attrib.get("value") for p in node.findall("param")}
        args = {a.attrib["name"]: a.attrib.get("default") for a in self.root.findall("arg")}
        self.assertEqual(args["map_file"], "/home/nrc/catkin_ws/maps/GlobalMap.pcd")
        self.assertEqual(params["globalmap_dir"], "$(arg map_file)")
        self.assertEqual(params["require_base_ready"], "true")
        self.assertEqual(args["base_ready_topic"], "/ducted/system/ready")

    def test_public_topics_are_stable(self):
        remaps = {r.attrib["from"]: r.attrib["to"] for r in self.root.findall(".//remap")}
        expected = {
            "/Odometry": "/ducted/localization/odom",
            "/path": "/ducted/localization/path",
            "/Odometry_relocal": "/ducted/relocalization/scan_matching_odom",
            "/gobal_map_relocal": "/ducted/relocalization/global_map",
            "/local_map_relocal": "/ducted/relocalization/local_map",
            "/registed_current_scan": "/ducted/relocalization/registered_scan",
            "/initialpose": "/ducted/relocalization/initialpose",
            "/relocalization_ready": "/ducted/relocalization/ready",
        }
        for source, target in expected.items():
            self.assertEqual(remaps.get(source), target)

    def test_absolute_height_and_internal_extrinsic(self):
        self.assertEqual(self.config["frames"], {"map": "map", "body": "livox_frame"})
        self.assertFalse(self.config["height"]["subtract_ground_height"])
        self.assertEqual(
            self.config["mapping"]["extrinsic_T"],
            [-0.011, -0.02329, 0.04412],
        )

    def test_old_air_ground_nodes_and_extrinsic_are_absent(self):
        for forbidden in ("0.4567", "fix_odom_sam", "trans_imu", "map_remove_head"):
            self.assertNotIn(forbidden, self.launch_text)


class RelocalizationPortTest(unittest.TestCase):
    def test_driver_frames_gate_and_map_validation_are_ported(self):
        pkg = SRC / "sfast_lio"
        source = (pkg / "src" / "laserMapping_relocalization.cpp").read_text(encoding="utf-8")
        matching = (pkg / "include" / "matching" / "matching.hpp").read_text(encoding="utf-8")
        manifest = (pkg / "package.xml").read_text(encoding="utf-8")
        self.assertIn("livox_ros_driver2", manifest)
        self.assertIn("p_pre->lidar_type == AVIA || p_pre->lidar_type == MID360", source)
        self.assertIn('private_nh.param<bool>("require_base_ready"', source)
        self.assertIn('nh.param<std::string>("frames/map"', source)
        self.assertIn('nh.param<std::string>("frames/body"', source)
        self.assertIn("pose_msg->header.frame_id != mapFrame", source)
        self.assertIn("map_path_.front() != '/'", matching)
        self.assertIn("failed to load non-empty global map", matching)


if __name__ == "__main__":
    unittest.main()
