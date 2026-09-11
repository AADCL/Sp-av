#!/usr/bin/env python3
"""Parse all module selections without starting a ROS master or any nodes."""
import itertools
import json
from pathlib import Path

import rospkg
from roslaunch.config import load_config_default


def main():
    package = Path(rospkg.RosPack().get_path('ducted_bringup'))
    launch = str(package / 'launch/modules.launch')
    flags = ('start_external_odometry', 'start_terrain', 'start_rc_processing',
             'start_flight', 'start_avoidance', 'start_mission', 'start_recording')
    gates = ('input_contract_confirmed', 'reference_confirmed', 'mapping_confirmed',
             'geometry_confirmed', 'enable_flight_output', 'enable_output', 'enable_commands')
    checked = 0
    for localization in ('none', 'mapping', 'relocalization'):
        for values in itertools.product((False, True), repeat=len(flags)):
            selected = dict(zip(flags, values))
            args = ['localization:=' + localization]
            args += [name + ':=' + str(value).lower() for name, value in selected.items()]
            config = load_config_default([(launch, args)], None, verbose=False)
            names = [node.name for node in config.nodes]
            assert len(names) == len(set(names)), (args, 'duplicate node names')
            assert not any(node.package in ('mavros', 'livox_ros_driver2')
                           or node.type == 'base_health_monitor.py' for node in config.nodes), args
            assert sum(node.type == 'rc_monitor.py' for node in config.nodes) == int(
                selected['start_rc_processing'] or selected['start_flight']), args
            assert names.count('ducted_navigation') == int(selected['start_avoidance']), args
            assert names.count('ego') == int(selected['start_avoidance']), args
            assert names.count('mapping') == int(localization == 'mapping'), args
            assert names.count('relocalization') == int(localization == 'relocalization'), args
            assert names.count('localization_frames') == int(localization != 'none'), args
            for node in config.nodes:
                if node.name == 'localization_frames':
                    assert dict(node.remap_args)['output'] == '/ducted/localization/body_odom', args
                if node.name == 'external_odometry_relay':
                    assert node.type == 'external_odometry_relay.py', args
            for name, param in config.params.items():
                if name == '/localization_frames/input_contract_confirmed':
                    assert param.value is True, args  # Local conversion, not a PX4 sender.
                    continue
                if name.rsplit('/', 1)[-1] in gates:
                    assert param.value is False, (args, name, param.value)
            checked += 1
    try:
        load_config_default([(launch, ['localization:=invalid'])], None, verbose=False)
    except Exception:
        invalid_rejected = True
    else:
        raise AssertionError('invalid localization choice was accepted')
    report = dict(passed=True, combinations=checked, invalid_choice_rejected=invalid_rejected,
                  scope='launch resolution only; dependency freshness is checked at runtime')
    output = package.parents[1] / 'logs/launch_matrix_result.json'
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
