#!/usr/bin/env python3
"""Isolated fake MAVROS; stall injection exists only in the non-installed test binary."""
import json
import os
import socket
import subprocess
import time
from xmlrpc.client import ServerProxy
import offboard_isolated as h

PORT = 11329
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % PORT
h.LOG_ROOT = h.ROOT / 'logs' / 'offboard_split_isolated'


def main():
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', PORT)) == 0:
            raise RuntimeError('isolated test port is occupied')
    processes, checks = [], []
    master = h.launch(['roscore', '-p', str(PORT)], 'master')
    processes.append(master)
    try:
        h.wait_for(lambda: ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0] == 1,
                   10, 'isolated master')
        h.rospy.init_node('split_threads_test', disable_signals=True)
        fake, sink = h.FakeMavros(), h.StatusSink()
        mission = h.rospy.Publisher('/ctrl_cmd/waypoints', h.NavPath, queue_size=1, latch=True)
        h.rospy.set_param('/ctrl_cmd/state', 0)
        command = h.node_command(horizontal_speed=.3, waypoint_timeout=30)
        command[2] = 'offboard_controller_test_node'
        node = h.launch(command, 'split_node')
        processes.append(node)
        h.wait_for(lambda: sink.phase('IDLE_GROUND'), 4, 'idle')
        h.wait_for(lambda: mission.get_num_connections() == 1, 3, 'mission subscriber')
        h.publish_sequence(mission, [(4., 0., .35, 0.)])
        h.wait_for(lambda: sink.latest and sink.latest.mission_loaded, 3, 'mission loaded')
        system = ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/split_test')[2]
        publishers = dict(system[0]).get('/mavros/setpoint_position/local', [])
        assert publishers == ['/ducted_offboard_controller'], publishers
        assert not any(topic in dict(system[1]) for topic in ('/ducted/control/target', '/ducted/navigation/status'))
        assert '/ctrl_cmd/planner_context' not in dict(system[0])
        checks.append('one MAVROS setpoint publisher; no planning subscriptions or permission output')
        h.rospy.set_param('/ctrl_cmd/state', 1)
        h.wait_for(lambda: sink.phase('HOLDING'), 6, 'takeoff then hold')
        h.rospy.set_param('/ctrl_cmd/state', 2)
        h.wait_for(lambda: sink.phase('GUIDING'), 3, 'guiding')
        time.sleep(.35)
        started = time.monotonic()
        h.rospy.set_param('/ducted_offboard_controller/test_task_stall_seconds', 1.4)
        stale = h.wait_for(lambda: sink.latest if sink.latest and
                          sink.latest.task_heartbeat_age > .5 and sink.latest.phase == 'PAUSED'
                          else None, 3, 'executor hover during stalled task thread')
        assert stale.task_phase == 'GUIDING' and stale.executor_phase == 'PAUSED'
        captured = tuple(fake.position)
        h.publish_sequence(mission, [(.7, .2, .35, 0.)])
        time.sleep(1.0)
        assert sink.phase('PAUSED') and h.math.dist(captured, tuple(fake.position)) < .08
        assert sink.latest.task_heartbeat_age < .3
        with fake.lock:
            times = [t for t in fake.setpoint_times if started <= t <= started + 1.4]
        assert len(times) >= 22, len(times)
        maximum_gap = max(b-a for a, b in zip(times, times[1:]))
        assert maximum_gap < .2, maximum_gap
        checks.append('stalled task thread: executor keeps 20 Hz, expires lease, captures hover')
        checks.append('task recovery and replacement mission cannot automatically resume flight')
        h.rospy.set_param('/ctrl_cmd/state', 0)
        h.wait_for(lambda: sink.latest.command == 0, 2, 'zero observed')
        h.rospy.set_param('/ctrl_cmd/state', 2)
        h.wait_for(lambda: sink.phase('GUIDING'), 3, 'explicit resumed task')
        h.wait_for(lambda: sink.phase('HOLDING') and 'mission complete' in sink.latest.reason,
                   8, 'replacement task completion')
        checks.append('explicit resume executes replacement task then holds')
        h.rospy.set_param('/ctrl_cmd/state', 3)
        h.wait_for(lambda: sink.phase('LANDING'), 3, 'native landing confirmed')
        h.rospy.set_param('/ducted_offboard_controller/test_task_stall_seconds', 1.4)
        h.wait_for(lambda: not fake.armed, 5, 'executor lands and disarms without task heartbeat')
        checks.append('confirmed landing continues and normally disarms during task stall')
        h.stop(node); processes.remove(node)

        refusals = [
            ['roslaunch', 'ducted_bringup', name] for name in
            ('ego_offboard.launch', 'ego_planner.launch', 'local_avoidance.launch')]
        refusals += [
            ['roslaunch', 'ducted_bringup', 'modules.launch', 'start_avoidance:=true'],
            ['rosrun', 'ducted_planning', 'ego_local_planner'],
            ['rosrun', 'ducted_navigation', 'ducted_navigation_node.py'],
            ['rosrun', 'ducted_offboard', 'offboard_controller_node', '_execution_mode:=ego']]
        for index, args in enumerate(refusals):
            result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, timeout=15, env=os.environ.copy())
            (h.LOG_ROOT / ('frozen_%d.log' % index)).write_text(result.stdout)
            assert 'EGO is frozen' in result.stdout, (args, result.stdout)
            # roslaunch may itself return zero after its required guard exits 2.
            if args[0] == 'rosrun':
                assert result.returncode != 0
        system = ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/split_test')[2]
        assert '/mavros/setpoint_position/local' not in dict(system[0])
        assert '/ducted/navigation/command' not in dict(system[2])
        checks.append('seven frozen launch/direct-node entries refuse without planner or controller startup')
        return {'passed': True, 'checks': checks, 'check_count': len(checks),
                'stall_setpoints': len(times), 'stall_max_gap_seconds': maximum_gap,
                'scope': 'isolated fake MAVROS only; no hardware commands'}
    finally:
        for process in reversed(processes):
            h.stop(process)


if __name__ == '__main__':
    result_path = h.LOG_ROOT / 'result.json'
    try:
        result = main()
    except Exception as error:
        result_path.parent.mkdir(parents=True, exist_ok=True)
        result_path.write_text(json.dumps({'passed': False, 'error': str(error)}, indent=2)+'\n')
        raise
    result_path.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
