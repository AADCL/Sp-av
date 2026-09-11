#!/usr/bin/env python3
"""Read-only startup inspection. Exit 0 only when requested checks pass."""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time
from xmlrpc.client import ServerProxy, Transport

import rospy
import tf2_ros
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from ducted_msgs.msg import TerrainHeight


class ProbeTransport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2.
        return connection


def master_call(uri, method, caller_id):
    with ServerProxy(uri, transport=ProbeTransport()) as master:
        return getattr(master, method)(caller_id)


def emit_report(report, output):
    if output:
        try:
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        except OSError as error:
            report = dict(report, passed=False, output_error=str(error))
    encoded = json.dumps(report, indent=2, ensure_ascii=False)
    print(encoded)
    return 0 if report['passed'] else 1


def error_report(profile, error, output):
    return emit_report(
        {'profile': profile, 'timestamp': time.time(), 'checks': [], 'passed': False,
         'error': str(error), 'scope': 'startup dependencies, not flight/geometry acceptance'},
        output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('base', 'localization', 'terrain'), default='base')
    parser.add_argument('--duration', type=float, default=3.)
    parser.add_argument('--serial', default='/dev/ttyTHS0')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(rospy.myargv()[1:])
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 30:
        return error_report(args.profile, 'duration must be between 1 and 30 seconds', args.output)
    initialized = False
    try:
        uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
        code, detail, _ = master_call(uri, 'getPid', '/ducted_startup_probe')
        if code != 1:
            raise RuntimeError('ROS master rejected getPid: '+detail)
        rospy.init_node('ducted_startup_check', anonymous=True, disable_signals=True)
        initialized = True
        lock = threading.Lock()
        samples = {}
        def callback(topic):
            def receive(message):
                with lock:
                    entry = samples.setdefault(topic, {'count': 0})
                    entry.update(message=message, wall=time.monotonic(), count=entry['count']+1)
            return receive
        types = {'/mavros/state': State, '/mavros/rc/in': RCIn,
                 '/livox/lidar': rospy.AnyMsg, '/ducted/system/ready': Bool}
        if args.profile != 'base':
            types['/ducted/localization/odom'] = Odometry
        if args.profile == 'terrain':
            types['/ducted/terrain/height'] = TerrainHeight
            types['/ducted/terrain/ready'] = Bool
        subscribers = [rospy.Subscriber(t, kind, callback(t), queue_size=1)
                       for t, kind in types.items()]
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        internal_buffer = tf2_ros.Buffer()
        def internal_tf(message):
            for transform in message.transforms:
                internal_buffer.set_transform_static(transform, 'mavros_internal')
        internal_subscription = rospy.Subscriber('/mavros/internal_tf_static', TFMessage, internal_tf, queue_size=1)
        time.sleep(args.duration)
        with lock:
            snapshot = {topic: dict(entry) for topic, entry in samples.items()}
        now, wall = rospy.get_time(), time.monotonic()
        checks = []
        def check(name, passed, detail):
            checks.append({'name': name, 'passed': bool(passed), 'detail': detail})
        def recent(topic, limit):
            sample = snapshot.get(topic)
            return sample and 0 <= wall - sample['wall'] <= limit
        def source_recent(message, limit):
            stamp = message.header.stamp.to_sec()
            return math.isfinite(stamp) and stamp > 0 and -.05 <= now - stamp <= limit
        check('ros_clock', now > 1704067200 and not rospy.get_param('/use_sim_time', False),
              {'ros_time': now, 'scope': 'real hardware startup'})
        check('serial_permissions', os.access(args.serial, os.R_OK | os.W_OK), args.serial)
        for topic in types:
            sample = snapshot.get(topic)
            check('stream:' + topic, recent(topic, 1.),
                  {'samples': sample['count'] if sample else 0})
        state = snapshot.get('/mavros/state', {}).get('message')
        check('fcu_connected', state is not None and state.connected,
              str(state) if state is not None else 'missing')
        ready = snapshot.get('/ducted/system/ready', {}).get('message')
        check('base_ready', ready is not None and ready.data
              and recent('/ducted/system/ready', 1.), '')
        rc = snapshot.get('/mavros/rc/in', {}).get('message')
        check('rc_transport', rc is not None and source_recent(rc, .5)
              and len(rc.channels) >= 12 and all(500 <= p <= 2500 for p in rc.channels[:12])
              and rc.rssi != 0,
              {'channels': list(rc.channels) if rc is not None else [],
               'rssi': rc.rssi if rc is not None else None,
               'scope': 'transport only; physical role mapping and kill are separate flight gates'})
        for parent, child in [('odom', 'odom_ned'), ('base_link', 'base_link_frd')]:
            check('tf:' + parent + '->' + child,
                  internal_buffer.can_transform(parent, child, rospy.Time(0), rospy.Duration(0)), 'MAVROS private conversion TF')
        code, detail, system_state = master_call(uri, 'getSystemState', rospy.get_name())
        if code != 1:
            raise RuntimeError('ROS master rejected getSystemState: '+detail)
        publishers, _, _ = system_state
        odom_publishers = dict(publishers).get('/ducted/localization/odom', [])
        check('localization_exclusive', len(odom_publishers) <= 1, odom_publishers)
        if args.profile != 'base':
            msg = snapshot.get('/ducted/localization/odom', {}).get('message')
            finite_pose = False
            if msg is not None:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                finite_pose = (all(math.isfinite(v) for v in
                                   (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                               and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) < .01)
            check('localization_source', msg is not None and source_recent(msg, .25)
                  and msg.header.frame_id == 'camera_init' and msg.child_frame_id == 'body'
                  and finite_pose and len(odom_publishers) == 1, odom_publishers)
        if args.profile == 'terrain':
            msg = snapshot.get('/ducted/terrain/height', {}).get('message')
            check('terrain_valid', msg is not None and msg.valid and source_recent(msg, .5)
                  and msg.header.frame_id == 'odom'
                  and all(math.isfinite(v) for v in (msg.ground_z, msg.agl, msg.variance)),
                  str(msg) if msg is not None else 'missing')
            msg = snapshot.get('/ducted/terrain/ready', {}).get('message')
            check('terrain_ready', msg is not None and msg.data
                  and recent('/ducted/terrain/ready', .5), '')
        report = {'profile': args.profile, 'timestamp': time.time(), 'checks': checks,
                  'passed': all(c['passed'] for c in checks),
                  'scope': 'startup dependencies, not flight/geometry acceptance'}
        return emit_report(report, args.output)
    except Exception as error:
        return error_report(args.profile, error, args.output)
    finally:
        if initialized:
            rospy.signal_shutdown('inspection complete')


if __name__ == '__main__':
    raise SystemExit(main())
