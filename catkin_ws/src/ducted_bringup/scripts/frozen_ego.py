#!/usr/bin/env python3
"""Deliberate runtime refusal; no ROS publishers, services or planner are created."""
import sys
if __name__ == '__main__':
    sys.stderr.write('EGO 已冻结 (EGO is frozen). 此入口不会启动规划或飞控节点；请使用 ducted_offboard/offboard_control.launch。\n')
    sys.exit(2)
