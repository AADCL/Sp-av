#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "src"))

try:
    from ducted_bringup.health import BaseHealth
except ImportError:
    BaseHealth = None


class BaseHealthAvailabilityTest(unittest.TestCase):
    def test_health_evaluator_exists(self):
        self.assertIsNotNone(BaseHealth)


@unittest.skipIf(BaseHealth is None, "health evaluator is not implemented")
class BaseHealthTest(unittest.TestCase):
    def test_required_inputs_begin_not_ready(self):
        health = BaseHealth(require_mavros=True, require_livox=True, lidar_timeout_sec=2.0)

        result = health.evaluate(now=10.0)

        self.assertFalse(result.ready)
        self.assertEqual(result.reasons, ("mavros state missing", "livox points missing"))

    def test_connected_mavros_and_fresh_livox_are_ready(self):
        health = BaseHealth(
            require_mavros=True,
            require_livox=True,
            mavros_timeout_sec=2.0,
            lidar_timeout_sec=2.0,
        )
        health.update_mavros(connected=True, now=9.0)
        health.update_livox(now=9.0)

        result = health.evaluate(now=10.0)

        self.assertTrue(result.ready)
        self.assertEqual(result.reasons, ())

    def test_disconnected_mavros_blocks_readiness(self):
        health = BaseHealth(require_mavros=True, require_livox=False, lidar_timeout_sec=2.0)
        health.update_mavros(connected=False, now=9.0)

        result = health.evaluate(now=10.0)

        self.assertFalse(result.ready)
        self.assertEqual(result.reasons, ("mavros disconnected",))

    def test_stale_mavros_state_blocks_readiness(self):
        health = BaseHealth(
            require_mavros=True,
            require_livox=False,
            mavros_timeout_sec=2.0,
            lidar_timeout_sec=2.0,
        )
        health.update_mavros(connected=True, now=7.0)

        result = health.evaluate(now=10.0)

        self.assertFalse(result.ready)
        self.assertEqual(result.reasons, ("mavros state stale",))

    def test_stale_livox_data_blocks_readiness(self):
        health = BaseHealth(require_mavros=False, require_livox=True, lidar_timeout_sec=2.0)
        health.update_livox(now=7.0)

        result = health.evaluate(now=10.0)

        self.assertFalse(result.ready)
        self.assertEqual(result.reasons, ("livox points stale",))

    def test_disabled_inputs_do_not_block_readiness(self):
        health = BaseHealth(require_mavros=False, require_livox=False, lidar_timeout_sec=2.0)

        result = health.evaluate(now=10.0)

        self.assertTrue(result.ready)
        self.assertEqual(result.reasons, ())


if __name__ == "__main__":
    unittest.main()
