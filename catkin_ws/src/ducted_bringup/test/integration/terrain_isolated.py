#!/usr/bin/env python3
"""Exercise production terrain node against synthetic sensors on its own master."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11322'
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ.pop('ROS_IP', None)
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header
from ducted_msgs.msg import TerrainHeight

ROOT = Path('/home/nrc/catkin_ws')
LOG = ROOT/'logs/terrain_isolated'
LOG.mkdir(exist_ok=True)
children = []
handles = []
checks = []
latest = []
events = []

def launch(command, name):
    handle = (LOG/(name+'.log')).open('w')
    handles.append(handle)
    child = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                             start_new_session=True)
    children.append(child)
    return child

def stop(child):
    if child.poll() is None:
        os.killpg(child.pid, signal.SIGINT)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait(timeout=3)

def observe(msg):
    latest[:] = [msg]
    events.append((time.monotonic(), msg.valid))

def check(name, value, detail=None):
    checks.append({'name': name, 'passed': bool(value), 'detail': detail})
    print(checks[-1], flush=True)
    if not value:
        raise AssertionError(name)

try:
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 11322)) == 0:
            raise RuntimeError('test port11322 already in use; refusing to share a master')
    master = launch(['roscore', '-p', '11322'], 'master')
    from xmlrpc.client import ServerProxy
    deadline = time.monotonic()+8
    while True:
        try:
            with ServerProxy(os.environ['ROS_MASTER_URI']) as server:
                if server.getPid('/terrain_test')[0] == 1:
                    break
        except OSError:
            pass
        if time.monotonic() > deadline:
            raise RuntimeError('isolated master did not start')
        time.sleep(.1)
    rospy.init_node('terrain_isolated_harness', disable_signals=True)
    pubs = {
        'base': rospy.Publisher('/ducted/system/ready', Bool, queue_size=1),
        'odom': rospy.Publisher('/ducted/localization/odom', Odometry, queue_size=1),
        'cloud': rospy.Publisher('/ducted/relocalization/registered_scan', PointCloud2, queue_size=1),
    }
    sub = rospy.Subscriber('/ducted/terrain/height', TerrainHeight, observe, queue_size=10)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    # Tilt raw map 0.3 radians while operational ground remains horizontal.
    angle = .3
    transforms = []
    for parent, child, xyz, quat in [
            ('odom', 'map', (0., 0., 0.), (0., math.sin(angle/2), 0., math.cos(angle/2))),
            ('odom', 'base_link', (0., 0., 1.5), (0., 0., 0., 1.))]:
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
        transforms.append(t)
    broadcaster.sendTransform(transforms)

    def sample(points=None, odom=True, cloud=True, base=True, frame='map', skew=0.):
        stamp = rospy.Time.now()
        if base:
            pubs['base'].publish(True)
        if odom:
            m = Odometry()
            m.header.stamp = stamp
            m.header.frame_id, m.child_frame_id = 'map', 'livox_frame'
            m.pose.pose.orientation.w = 1.
            pubs['odom'].publish(m)
        time.sleep(.012)
        if cloud:
            pts = points if points is not None else flat
            # Inverse of odom<-map rotation, including all XYZ components.
            c, s = math.cos(angle), math.sin(angle)
            raw = [(c*x-s*z, y, s*x+c*z) for x,y,z in pts]
            h = Header(stamp=stamp+rospy.Duration(skew), frame_id=frame)
            pubs['cloud'].publish(point_cloud2.create_cloud_xyz32(h, raw))
        time.sleep(.065)

    flat = [(x*.08, y*.08, .4) for x in range(-7, 8) for y in range(-7, 8)]
    def run_scene(points=flat, seconds=.8, **kwargs):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            sample(points, **kwargs)

    node = launch(['roslaunch', '--skip-log-check', 'ducted_bringup', 'terrain_height.launch'], 'default_gate')
    run_scene(seconds=5.)
    check('default_geometry_gate', bool(latest) and not latest[-1].valid)
    stop(node)
    node = launch(['roslaunch', '--skip-log-check', 'ducted_bringup', 'terrain_height.launch',
                   'reference_confirmed:=true'], 'confirmed_synthetic_geometry')
    run_scene(seconds=5.)
    check('flat_tilted_map', latest[-1].valid and abs(latest[-1].agl-1.1)<.015
          and latest[-1].header.frame_id == 'odom', str(latest[-1]))
    slope = [(x*.08, y*.08, .4+.3*x*.08) for x in range(-7,8) for y in range(-7,8)]
    run_scene(slope)
    check('slope', latest[-1].valid and abs(latest[-1].agl-1.1)<.02)
    for z, name in [(.2, 'lower_step_side'), (.6, 'upper_step_side')]:
        run_scene([(x,y,z) for x,y,_ in flat])
        check(name, latest[-1].valid and abs(latest[-1].agl-(1.5-z))<.02)
    step = [(x*.08, y*.08, .4 if x+y<1 else .7)
            for x in range(-7,8) for y in range(-7,8)]
    step += [(x*.08, .2, 1.2) for x in range(6)]
    run_scene(step)
    check('ambiguous_step_with_obstacles', not latest[-1].valid)
    run_scene()
    check('recovery_after_step', latest[-1].valid)
    run_scene(cloud=False)
    check('cloud_dropout', not latest[-1].valid)
    run_scene()
    check('cloud_recovery', latest[-1].valid)
    run_scene(odom=False)
    check('odom_dropout', not latest[-1].valid)
    run_scene()
    run_scene(frame='wrong_map')
    check('wrong_frame', not latest[-1].valid)
    run_scene()
    run_scene(skew=-.12)
    check('source_skew', not latest[-1].valid)
    run_scene()
    run_scene(base=False, seconds=1.3)
    check('base_dropout', not latest[-1].valid)
    run_scene()
    check('base_recovery', latest[-1].valid)
    check('no_flight_commands', not any(t.startswith('/mavros/setpoint')
          for t,_ in rospy.get_published_topics()))
except Exception as error:
    checks.append({'name': 'exception', 'passed': False, 'detail': repr(error)})
finally:
    for child in reversed(children):
        stop(child)
    for handle in handles:
        handle.close()
    result = {'passed': bool(checks) and all(c['passed'] for c in checks),
              'scope': 'synthetic ROS integration only; no physical geometry acceptance',
              'checks': checks, 'children_stopped': all(c.poll() is not None for c in children)}
    (LOG/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    if not result['passed']:
        raise SystemExit(1)
