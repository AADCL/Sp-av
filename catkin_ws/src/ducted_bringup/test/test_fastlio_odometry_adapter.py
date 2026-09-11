#!/usr/bin/env python3
import math
import pathlib
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml


PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / "src"))

try:
    from ducted_bringup.fastlio_odometry import (
        AdapterError,
        adapt_odometry_values,
        quaternion_from_rpy,
    )
except ImportError:
    AdapterError = None
    adapt_odometry_values = None
    quaternion_from_rpy = None


class AdapterAvailabilityTest(unittest.TestCase):
    def test_adapter_exists(self):
        self.assertIsNotNone(adapt_odometry_values)


@unittest.skipIf(adapt_odometry_values is None, "adapter is not implemented")
class AdapterMathTest(unittest.TestCase):
    def test_sensor_pose_is_shifted_to_base_origin(self):
        result = adapt_odometry_values(
            position=(1.13, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            linear_velocity_map=(1.0, 0.0, 0.0),
            angular_velocity_sensor=(0.0, 0.0, 0.0),
            base_to_sensor_translation=(0.13, 0.0, 0.0),
            base_to_sensor_orientation=(0.0, 0.0, 0.0, 1.0),
        )
        for actual, expected in zip(result.position, (1.0, 2.0, 3.0)):
            self.assertAlmostEqual(actual, expected, places=12)
        for actual, expected in zip(result.linear_velocity_base, (1.0, 0.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=12)
        for actual, expected in zip(result.child_orientation, (0.0, 0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_orientation_and_map_velocity_are_converted_to_base(self):
        half = math.sqrt(0.5)
        result = adapt_odometry_values(
            position=(0.0, 0.0, 0.0),
            orientation=(0.0, 0.0, half, half),
            linear_velocity_map=(0.0, 2.0, 0.0),
            angular_velocity_sensor=(0.0, 0.0, 1.0),
            base_to_sensor_translation=(0.0, 0.0, 0.0),
            base_to_sensor_orientation=(0.0, 0.0, 0.0, 1.0),
        )
        self.assertAlmostEqual(result.linear_velocity_base[0], 2.0, places=6)
        self.assertAlmostEqual(result.linear_velocity_base[1], 0.0, places=6)
        self.assertAlmostEqual(result.angular_velocity_base[2], 1.0, places=6)

    def test_base_orientation_removes_sensor_mount_rotation(self):
        mount = quaternion_from_rpy(0.03, 0.4567, 0.0)
        result = adapt_odometry_values(
            position=(0.13, 0.0, 0.0),
            orientation=mount,
            linear_velocity_map=(0.0, 0.0, 0.0),
            angular_velocity_sensor=(0.0, 0.0, 0.0),
            base_to_sensor_translation=(0.13, 0.0, 0.0),
            base_to_sensor_orientation=mount,
        )
        self.assertAlmostEqual(result.position[0], 0.0, places=6)
        self.assertAlmostEqual(result.position[1], 0.0, places=6)
        self.assertAlmostEqual(result.position[2], 0.0, places=6)
        self.assertAlmostEqual(result.child_orientation[3], 1.0, places=6)

    def test_zero_quaternion_is_rejected(self):
        with self.assertRaises(AdapterError):
            adapt_odometry_values(
                position=(0.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 0.0),
                linear_velocity_map=(0.0, 0.0, 0.0),
                angular_velocity_sensor=(0.0, 0.0, 0.0),
                base_to_sensor_translation=(0.0, 0.0, 0.0),
                base_to_sensor_orientation=(0.0, 0.0, 0.0, 1.0),
            )


class AdapterContractTest(unittest.TestCase):
    def test_launch_has_required_topics_and_readiness_gates(self):
        root = ET.parse(PKG / "launch" / "external_odometry.launch").getroot()
        node = root.find(".//node[@type='external_odometry_relay.py']")
        self.assertIsNotNone(node)
        self.assertEqual(node.attrib.get("required"), "true")
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
        remaps = {r.attrib["from"]: r.attrib["to"] for r in node.findall("remap")}
        params = {p.attrib["name"]: p.attrib.get("value") for p in node.findall("param")}
        self.assertEqual(args["input_odom"], "/ducted/localization/body_odom")
        self.assertEqual(args["output_odom"], "/mavros/odometry/out")
        self.assertEqual(params["require_base_ready"], "true")
        self.assertEqual(params["require_localization_ready"], "true")
        self.assertEqual(remaps["input"], "$(arg input_odom)")
        self.assertEqual(remaps["output"], "$(arg output_odom)")
        self.assertIsNone(root.find(".//node[@type='static_transform_publisher']"))

    def test_localization_launches_own_frames_without_external_output(self):
        for name in ("mapping", "relocalization"):
            root = ET.parse(PKG / "launch" / (name + ".launch")).getroot()
            includes = [n.attrib['file'] for n in root.findall('include')]
            self.assertIn('$(find ducted_bringup)/launch/localization_frames.launch', includes)
        root = ET.parse(PKG / 'launch/localization_frames.launch').getroot()
        node = root.find("node[@type='fastlio_odometry_adapter.py']")
        self.assertIsNotNone(node)
        remaps = {r.attrib['from']: r.attrib['to'] for r in node.findall('remap')}
        self.assertEqual(remaps['output'], '/ducted/localization/body_odom')
        self.assertEqual(remaps['ready'], '/ducted/localization/frames_ready')

    def test_reused_extrinsic_is_declared_with_provenance(self):
        config = yaml.safe_load((PKG / "config" / "external_odometry.yaml").read_text())
        extrinsic = config["extrinsic"]["base_to_livox"]
        self.assertEqual(extrinsic["translation"], [0.13, 0.0, 0.0])
        self.assertEqual(extrinsic["rpy"], [0.03, 0.4567, 0.0])
        self.assertEqual(extrinsic["status"], "reused_unverified")
        self.assertIn("spirit-wing_v1.0", extrinsic["source"])
        self.assertFalse(config["input_contract_confirmed"])


if __name__ == "__main__":
    unittest.main()
