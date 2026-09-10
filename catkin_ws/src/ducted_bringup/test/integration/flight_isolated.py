#!/usr/bin/env python3
"""Production flight node against fake FCU/RC on a dedicated ROS master."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
from xmlrpc.client import ServerProxy

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11323'
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ.pop('ROS_IP', None)
import rospy
from ducted_msgs.msg import FlightControlStatus, FlightSetpoint, RCState
from ducted_msgs.srv import FlightCommand
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

ROOT = Path('/home/nrc/catkin_ws')
LOG = ROOT/'logs/flight_isolated'
LOG.mkdir(exist_ok=True)
children, handles, checks = [], [], []
lock = threading.RLock()
done = threading.Event()
state = dict(mode='POSCTL', armed=True, system_status=4, rc_mode='command', kill=False,
             land=False, external=True, base=True, telemetry=True, follow=True,
             x=0., y=0., z=0., target=None, pose_frame='odom', mode_delay=0.,
             modes=[], outputs=[], statuses=[], current=None)

def launch(command, name):
    f = (LOG/(name+'.log')).open('w')
    handles.append(f)
    p = subprocess.Popen(command, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    children.append(p)
    return p

def stop(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(timeout=3)

def check(name, condition, detail=None):
    checks.append(dict(name=name, passed=bool(condition), detail=detail))
    print(checks[-1], flush=True)
    if not condition:
        raise AssertionError(name)

def status_cb(msg):
    with lock:
        state['current'] = msg
        state['statuses'].append((time.monotonic(), msg.state, msg.reason, msg.request_id))

def setpoint_cb(msg):
    with lock:
        state['target'] = msg
        state['outputs'].append((time.monotonic(), msg.pose.position.x,
                                 msg.pose.position.y, msg.pose.position.z))

def mode_cb(request):
    with lock:
        state['modes'].append((time.monotonic(), request.custom_mode))
        delay = state['mode_delay']
    time.sleep(delay)
    with lock:
        state['mode'] = request.custom_mode
    return SetModeResponse(mode_sent=True)

def wait_for(predicate, seconds=5.):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        with lock:
            if predicate():
                return True
        time.sleep(.03)
    return False

def has_status(*names):
    return state['current'] is not None and state['current'].state in names

try:
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 11323)) == 0:
            raise RuntimeError('test port11323 already in use; refusing to share a master')
    master = launch(['roscore', '-p', '11323'], 'master')
    end = time.monotonic()+8
    while True:
        try:
            with ServerProxy(os.environ['ROS_MASTER_URI']) as server:
                if server.getPid('/flight_test')[0] == 1:
                    break
        except OSError:
            pass
        if time.monotonic()>end:
            raise RuntimeError('master startup timeout')
        time.sleep(.1)
    rospy.init_node('flight_isolated_harness', disable_signals=True)
    pubs = {
        'fcu': rospy.Publisher('/mavros/state', State, queue_size=1),
        'pose': rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1),
        'landed': rospy.Publisher('/mavros/extended_state', ExtendedState, queue_size=1),
        'rc': rospy.Publisher('/ducted/rc/state', RCState, queue_size=1),
        'base': rospy.Publisher('/ducted/system/ready', Bool, queue_size=1),
        'external': rospy.Publisher('/ducted/external_odometry/ready', Bool, queue_size=1),
        'external_pose': rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=1),
        'goal': rospy.Publisher('/ducted/control/target', FlightSetpoint, queue_size=1),
    }
    service = rospy.Service('/mavros/set_mode', SetMode, mode_cb)
    subs = [rospy.Subscriber('/ducted/control/status', FlightControlStatus, status_cb, queue_size=100),
            rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, setpoint_cb, queue_size=100)]

    def telemetry():
        last_base = 0.
        while not done.wait(.02):
            with lock:
                if not state['telemetry']:
                    continue
                now = rospy.Time.now()
                if state['follow'] and state['mode'] == 'OFFBOARD' and state['target']:
                    p = state['target'].pose.position
                    state.update(x=p.x, y=p.y, z=p.z)
                if state['mode'] == 'AUTO.LAND':
                    state['z'] = max(0., state['z']-.02)
                    if state['z'] <= .001:
                        state['armed'] = False
                fcu = State()
                fcu.header.stamp = now
                fcu.connected, fcu.armed = True, state['armed']
                fcu.mode, fcu.system_status = state['mode'], state['system_status']
                pose = PoseStamped()
                pose.header.stamp, pose.header.frame_id = now, state['pose_frame']
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = state['x'],state['y'],state['z']
                pose.pose.orientation.w = 1.
                external_pose = Odometry()
                external_pose.header.stamp, external_pose.header.frame_id = now, 'odom'
                external_pose.child_frame_id = 'base_link'
                external_pose.pose.pose = pose.pose
                landed = ExtendedState()
                landed.header.stamp = now
                landed.landed_state = 1 if state['z'] <= .001 else 2
                rc = RCState()
                rc.header.stamp = now
                rc.valid, rc.mode, rc.kill_switch, rc.land_switch = True,state['rc_mode'],state['kill'],state['land']
                pubs['fcu'].publish(fcu)
                pubs['pose'].publish(pose)
                pubs['landed'].publish(landed)
                pubs['rc'].publish(rc)
                if state['external']:
                    pubs['external'].publish(True)
                    pubs['external_pose'].publish(external_pose)
                if state['base'] and time.monotonic()-last_base >= .5:
                    pubs['base'].publish(True)
                    last_base = time.monotonic()

    worker = threading.Thread(target=telemetry, daemon=True)
    worker.start()
    common = ['roslaunch', '--skip-log-check', 'ducted_bringup', 'flight_control.launch',
              'start_rc_monitor:=false']
    node = launch(common, 'default_gate')
    check('default_status_available', wait_for(lambda: state['current'] is not None, 8.))
    rospy.wait_for_service('/ducted/control/command', timeout=3)
    command = rospy.ServiceProxy('/ducted/control/command', FlightCommand)
    response = command('engage', PoseStamped())
    time.sleep(.4)
    check('default_output_gate', not response.accepted and not state['modes'] and not state['outputs'])
    stop(node)
    with lock:
        state['current'] = None
    node = launch(common+['enable_flight_output:=true'], 'enabled_fake_backend')
    check('enabled_status_available', wait_for(lambda: state['current'] is not None, 8.))
    time.sleep(.6)
    command = rospy.ServiceProxy('/ducted/control/command', FlightCommand)

    def engage():
        time.sleep(.15)
        response = command('engage', PoseStamped())
        check('explicit_engage_accepted', response.accepted, response.message)
        check('offboard_feedback_owned', wait_for(lambda: has_status('HOLD') and state['current'].ready, 5.),
              str(state['current']))

    engage()
    with lock:
        first_output = state['outputs'][0][0]
        offboard_request = next(t for t,m in state['modes'] if m == 'OFFBOARD')
    check('continuous_prestream', offboard_request-first_output >= .95)
    target = PoseStamped()
    target.header.stamp, target.header.frame_id = rospy.Time.now(), 'odom'
    target.pose.position.z, target.pose.orientation.w = .6, 1.
    response = command('takeoff', target)
    check('explicit_absolute_takeoff', response.accepted)
    check('takeoff_actual_pose_dwell', wait_for(lambda: has_status('HOLD') and state['z']>.52, 8.))

    def publish_goal(request_id='goal_a', x=.7):
        goal = FlightSetpoint()
        goal.header.stamp, goal.header.frame_id = rospy.Time.now(), 'odom'
        goal.request_id = request_id
        goal.pose.position.x, goal.pose.position.z, goal.pose.orientation.w = x,.6,1.
        pubs['goal'].publish(goal)

    end = time.monotonic()+1.5
    while time.monotonic()<end:
        publish_goal()
        time.sleep(.08)
    check('planner_id_ack_and_motion', has_status('TRACK') and state['current'].request_id == 'goal_a'
          and state['x']>.3)
    check('planner_dropout_grace', wait_for(lambda: has_status('HOLD_GRACE'), 1.))
    check('planner_dropout_posctl', wait_for(lambda: state['mode']=='POSCTL' and has_status('DISABLED'), 4.))
    mode_count = len(state['modes'])
    time.sleep(.5)
    check('no_automatic_reentry', len(state['modes']) == mode_count)
    engage()
    response = command('release', PoseStamped())
    check('explicit_release_mode', response.accepted and wait_for(lambda: state['mode']=='POSCTL', 2.))
    check('release_feedback_disabled', wait_for(lambda: has_status('DISABLED')))
    engage()
    with lock:
        state['pose_frame'] = 'wrong_frame'
    check('wrong_pose_frame_revokes', wait_for(lambda: has_status('DISABLED'), 1.))
    with lock:
        state.update(pose_frame='odom', mode='POSCTL')
    time.sleep(.2)
    engage()
    with lock:
        state['rc_mode'] = 'manual'
    check('rc_manual_handoff', wait_for(lambda: state['mode']=='POSCTL' and has_status('DISABLED'), 2.))
    with lock:
        state['rc_mode'] = 'command'
    time.sleep(.2)
    engage()
    with lock:
        state['external'] = False
    check('external_dropout_revokes', wait_for(lambda: has_status('DISABLED'), 1.2))
    with lock:
        state.update(external=True, mode='POSCTL')
    time.sleep(.2)
    engage()
    response = command('land', PoseStamped())
    check('explicit_land', response.accepted)
    check('land_actual_feedback', wait_for(lambda: has_status('DISABLED') and not state['armed'], 5.))
    with lock:
        state.update(mode='POSCTL', kill=True)
    check('kill_latches', wait_for(lambda: has_status('INHIBITED'), 1.))
    with lock:
        state['kill'] = False
    time.sleep(.2)
    response = command('reset', PoseStamped())
    check('explicit_disarmed_reset', response.accepted and wait_for(lambda: has_status('DISABLED')))
    with ServerProxy(os.environ['ROS_MASTER_URI']) as server:
        services = server.getSystemState('/flight_test')[2][2]
    check('no_arm_service', not any('/arming' in name for name,_ in services))
except Exception as error:
    checks.append(dict(name='exception', passed=False, detail=repr(error)))
finally:
    done.set()
    for p in reversed(children):
        stop(p)
    for f in handles:
        f.close()
    result = dict(passed=bool(checks) and all(c['passed'] for c in checks), checks=checks,
                  mode_calls=state['modes'], statuses=state['statuses'],
                  setpoint_count=len(state['outputs']),
                  children_stopped=all(p.poll() is not None for p in children),
                  scope='isolated fake FCU only; no real arming/mode/setpoint actions')
    (LOG/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='statuses'}, indent=2), flush=True)
    if not result['passed']:
        raise SystemExit(1)
