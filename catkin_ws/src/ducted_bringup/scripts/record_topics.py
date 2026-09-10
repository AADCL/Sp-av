#!/usr/bin/env python3
"""Record an explicitly requested session after the base heartbeat is live."""
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import rospy
from roslib.packages import find_node
from std_msgs.msg import Bool


def stop_process(process):
    if process.poll() is not None:
        return 'already exited'
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
        return 'stopped with SIGINT'
    except subprocess.TimeoutExpired:
        process.terminate()
    try:
        process.wait(timeout=3)
        return 'terminated after SIGINT timeout'
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        return 'killed after SIGINT and SIGTERM timeouts'


def main():
    rospy.init_node('ducted_recording')
    root = Path(rospy.get_param('~directory', '/home/nrc/catkin_ws/logs/recordings'))
    timeout = float(rospy.get_param('~base_timeout', 15.))
    split_mb = rospy.get_param('~split_mb', 512)
    topics = rospy.get_param('~topics')
    if (not math.isfinite(timeout) or timeout <= 0 or type(split_mb) is not int or split_mb < 1
            or not isinstance(topics, list) or not topics
            or any(not isinstance(t, str) or not t.startswith('/') or any(c.isspace() for c in t)
                   for t in topics)):
        raise ValueError('invalid recording timeout, split size, or topic list')
    wake = threading.Event()
    lock = threading.Lock()
    heartbeat = {'count': 0, 'wall': None}
    def base(message):
        with lock:
            now = time.monotonic()
            if not message.data:
                heartbeat['count'] = 0
            elif heartbeat['wall'] is None or now-heartbeat['wall'] > 1.:
                heartbeat['count'] = 1
            else:
                heartbeat['count'] += 1
            heartbeat['wall'] = now
    subscription = rospy.Subscriber(rospy.get_param('~base_ready_topic', '/ducted/system/ready'),
                                    Bool, base, queue_size=1)
    rospy.on_shutdown(wake.set)
    deadline = time.monotonic()+timeout
    while not rospy.is_shutdown():
        with lock:
            ready = (heartbeat['count'] >= 2 and time.monotonic()-heartbeat['wall'] <= 1.)
        if ready:
            break
        if time.monotonic() >= deadline:
            rospy.logfatal('Recording requires live base_system readiness heartbeats')
            return 2
        wake.wait(.05)
    if rospy.is_shutdown():
        return 0
    session = root/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
    session.mkdir(parents=True, exist_ok=False)
    command = ['--split', '--size='+str(split_mb), '--buffsize=64',
               '-O', str(session/'telemetry')]+topics
    metadata = {'started_utc': datetime.now(timezone.utc).isoformat(), 'topics': topics,
                'parameters': {}, 'base_required_at_start': True,
                'base_loss_behavior': 'continue recording fault evidence'}
    process = None
    result = None
    cleanup = 'not started'
    error = None
    try:
        # Launch the native recorder directly. The rosbag CLI is a Python
        # wrapper with its own child, which could survive killing the wrapper.
        executables = find_node('rosbag', 'record')
        if not executables:
            raise FileNotFoundError('rosbag/record executable unavailable')
        command.insert(0, executables[0])
        metadata['command'] = command
        # Record parameter values with the data so later tuning can be reproduced.
        metadata['parameters'] = {ns: rospy.get_param(ns, {}) for ns in
                                  ('/ducted', '/terrain_height', '/fastlio_odometry_adapter',
                                   '/rc_monitor', '/flight_controller', '/local_avoidance',
                                   '/mavros', '/common', '/preprocess', '/mapping',
                                   '/publish', '/relocalization')}
        (session/'session.json').write_text(json.dumps(metadata, indent=2, default=str)+'\n')
        rospy.loginfo('Recording telemetry under %s', session)
        with (session/'recorder.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            while not rospy.is_shutdown() and process.poll() is None:
                wake.wait(.1)
    except Exception as recorder_error:
        error = type(recorder_error).__name__+': '+str(recorder_error)
        rospy.logerr('Recording failed: %s', error)
    finally:
        if process is not None:
            try:
                cleanup = stop_process(process)
            except Exception as cleanup_error:
                cleanup = 'cleanup failed'
                detail = type(cleanup_error).__name__+': '+str(cleanup_error)
                error = detail if error is None else error+'; '+detail
                rospy.logerr('Recorder cleanup failed: %s', detail)
            result = process.returncode
        metadata.update(stopped_utc=datetime.now(timezone.utc).isoformat(),
                        recorder_exit=result, cleanup=cleanup)
        if error is not None:
            metadata['error'] = error
        try:
            (session/'session.json').write_text(json.dumps(metadata, indent=2, default=str)+'\n')
        except Exception as metadata_error:
            rospy.logerr('Could not finalize recording metadata: %s', metadata_error)
            error = error or type(metadata_error).__name__+': '+str(metadata_error)
    if result is None or error is not None:
        return 3
    # rosbag may return SIGINT after writing its index during a normal shutdown.
    return 0 if result in (0, -signal.SIGINT) else result


if __name__ == '__main__':
    raise SystemExit(main())
