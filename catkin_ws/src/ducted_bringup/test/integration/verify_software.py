#!/usr/bin/env python3
"""Run software acceptance; isolated ROS integration requires --integration."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--integration', action='store_true')
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    root = here.parents[3]
    directory = root / 'logs' / ('software_verification_' + time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir(parents=True, exist_ok=False)
    stages = [
        ('build', ['catkin_make', '-j3', '-l3'], 600),
        ('unit', ['catkin_make', 'run_tests_ducted_bringup', 'run_tests_ducted_control',
                  'run_tests_ducted_navigation', 'run_tests_ducted_offboard', '-j3', '-l3'], 300),
        ('results', ['catkin_test_results', 'build/test_results'], 30),
        ('launch_matrix', [sys.executable, str(here / 'launch_matrix.py')], 600),
        ('frozen_launch', [sys.executable, str(root / 'src/ducted_offboard/test/integration/frozen_launch_checks.py')], 60),
    ]
    if args.integration:
        stages += [('thread_test_build', ['catkin_make', 'offboard_controller_test_node', '-j3', '-l3'], 300)]
        stages += [(name, [sys.executable, str(here / (name + '.py'))], 240)
                   for name in ('terrain_isolated',)]
        stages += [(name, [sys.executable, str(root / 'src/ducted_offboard/test/integration' / (name+'.py'))], 240)
                   for name in ('offboard_isolated', 'split_threads_isolated')]
    results = []
    for name, command, timeout in stages:
        with (directory / (name + '.log')).open('w') as stream:
            process = subprocess.Popen(command, cwd=str(root), stdout=stream,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            expired = False
            try:
                code = process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                expired = True
                os.killpg(process.pid, signal.SIGINT)
                try:
                    code = process.wait(timeout=90)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    code = process.wait(timeout=5)
        results.append(dict(stage=name, exit_code=code, timeout=expired))
        print(results[-1], flush=True)
        if code or expired:
            break
    passed = len(results) == len(stages) and all(
        entry['exit_code'] == 0 and not entry['timeout'] for entry in results)
    report = dict(passed=passed, integration=args.integration, stages=results,
                  scope='software tests; physical calibration and flight acceptance excluded')
    (directory / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(str(directory / 'result.json'))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
