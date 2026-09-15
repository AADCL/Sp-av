#!/usr/bin/env python3
"""Resolve launch files without starting any node or ROS master."""
from pathlib import Path
import unittest
import rospkg
from roslaunch.config import load_config_default


class FrozenLaunches(unittest.TestCase):
    def test_frozen_entries_never_start_control_or_planning(self):
        bringup = Path(rospkg.RosPack().get_path('ducted_bringup'))
        for entry in ('ego_offboard.launch', 'ego_planner.launch', 'local_avoidance.launch'):
            with self.subTest(entry=entry):
                config = load_config_default([str(bringup/'launch'/entry)], None, verbose=False)
                forbidden = {'offboard_controller_node', 'ego_local_planner', 'ducted_navigation_node.py'}
                self.assertFalse(forbidden.intersection(n.type for n in config.nodes))
                self.assertTrue(config.nodes, 'frozen entry must explain why it cannot start')

    def test_mapping_and_relocalization_keep_their_nodes(self):
        bringup = Path(rospkg.RosPack().get_path('ducted_bringup'))
        for entry in ('base_system.launch', 'mapping.launch', 'relocalization.launch'):
            config = load_config_default([str(bringup/'launch'/entry)], None, verbose=False)
            self.assertTrue(config.nodes)
            self.assertFalse(any(n.package == 'ducted_offboard' for n in config.nodes))


if __name__ == '__main__':
    unittest.main()
