#!/usr/bin/env python3
"""Connect real planner, mission and flight nodes to an isolated fake vehicle."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
from xmlrpc.client import ServerProxy
import yaml

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11326'
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ.pop('ROS_IP', None)
import rospy
from ducted_msgs.msg import FlightControlStatus, FlightSetpoint, RCState, TerrainHeight
from ducted_msgs.srv import FlightCommand
from ducted_navigation.msg import NavigationStatus
from ducted_navigation.srv import Navigate
from ducted_mission.msg import MissionStatus
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header
from std_srvs.srv import Trigger

ROOT = Path('/home/nrc/catkin_ws')
LOG = ROOT/'logs/ego_mission_isolated'
LOG.mkdir(exist_ok=True)
children, handles, checks = [], [], []
done = threading.Event()
lock = threading.RLock()
data = dict(x=0., y=0., z=1.2, yaw=0., mode='POSCTL', rc_mode='command',
            target=None, flight=None, navigation=None, mission=None, cloud=True,
            obstacle=False, pole=False, follow=True, mode_calls=[], mission_history=[], telemetry_history=[],
            planner_targets=[], terrain_valid=True, velocity=(0., 0., 0.))

def step_translation(position, velocity, target, dt=.02):
    """Finite-acceleration position follower for the synthetic aircraft."""
    desired=tuple(6.*(target[i]-position[i]) for i in range(3))
    speed=math.sqrt(sum(v*v for v in desired))
    if speed>.4:
        desired=tuple(v*.4/speed for v in desired)
    change=tuple(desired[i]-velocity[i] for i in range(3))
    magnitude=math.sqrt(sum(v*v for v in change))
    ratio=min(1.,.5*dt/magnitude) if magnitude else 1.
    velocity=tuple(velocity[i]+ratio*change[i] for i in range(3))
    return tuple(position[i]+velocity[i]*dt for i in range(3)),velocity

def launch(args, name):
    f = (LOG/(name+'.log')).open('w')
    handles.append(f)
    p = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
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

def wait_for(predicate, seconds=6.):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        with lock:
            if predicate():
                return True
        time.sleep(.025)
    return False

def check(name, condition, detail=None):
    checks.append(dict(name=name, passed=bool(condition), detail=detail))
    print(checks[-1], flush=True)
    if not condition:
        raise AssertionError(name)

def receive(key):
    def callback(m):
        with lock:
            data[key] = m
            if key == 'mission':
                data['mission_history'].append((time.monotonic(), m.state, m.request_id, m.waypoint_index, m.reason))
            if key in ('navigation', 'flight'):
                data['telemetry_history'].append((time.monotonic(), key, m.state, m.request_id,
                                                  m.reason, data['x'], data['y'], data['z']))
    return callback

def planner_target_callback(message):
    with lock:
        data['planner_targets'].append((time.monotonic(), message.request_id))

def mode_callback(request):
    with lock:
        data['mode'] = request.custom_mode
        data['mode_calls'].append(request.custom_mode)
    return SetModeResponse(mode_sent=True)

def pose(x, y, z=1.2):
    m = PoseStamped()
    m.header.stamp, m.header.frame_id = rospy.Time.now(), 'odom'
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x,y,z
    m.pose.orientation.w = 1.
    return m

try:
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 11326)) == 0:
            raise RuntimeError('test port11326 already in use; refusing to share a master')
    launch(['roscore', '-p', '11326'], 'master')
    deadline = time.monotonic()+8
    while True:
        try:
            with ServerProxy(os.environ['ROS_MASTER_URI']) as server:
                if server.getPid('/navigation_test')[0] == 1:
                    break
        except OSError:
            pass
        if time.monotonic()>deadline:
            raise RuntimeError('master startup timeout')
        time.sleep(.1)
    rospy.init_node('ego_mission_harness', disable_signals=True)
    pubs = {
        'fcu': rospy.Publisher('/mavros/state', State, queue_size=1),
        'pose': rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1),
        'odom': rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=1),
        'landed': rospy.Publisher('/mavros/extended_state', ExtendedState, queue_size=1),
        'rc': rospy.Publisher('/ducted/rc/state', RCState, queue_size=1),
        'base': rospy.Publisher('/ducted/system/ready', Bool, queue_size=1),
        'external': rospy.Publisher('/ducted/external_odometry/ready', Bool, queue_size=1),
        'height': rospy.Publisher('/ducted/terrain/height', TerrainHeight, queue_size=1),
        'terrain_ready': rospy.Publisher('/ducted/terrain/ready', Bool, queue_size=1),
        'cloud': rospy.Publisher('/ducted/relocalization/registered_scan', PointCloud2, queue_size=1),
    }
    service = rospy.Service('/mavros/set_mode', SetMode, mode_callback)
    subs = [rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, receive('target'), queue_size=1),
            rospy.Subscriber('/ducted/control/status', FlightControlStatus, receive('flight'), queue_size=1),
            rospy.Subscriber('/ducted/navigation/status', NavigationStatus, receive('navigation'), queue_size=1),
            rospy.Subscriber('/ducted/control/target', FlightSetpoint, planner_target_callback, queue_size=20),
            rospy.Subscriber('/ducted/mission/status', MissionStatus, receive('mission'), queue_size=20)]

    def sensor_loop():
        last_base, last_cloud = 0., 0.
        while not done.wait(.02):
            with lock:
                old = tuple(data[k] for k in ('x','y','z'))
                if data['follow'] and data['mode']=='OFFBOARD' and data['target'] is not None:
                    target = data['target'].pose
                    position,data['velocity']=step_translation(
                        old,data['velocity'],tuple(getattr(target.position,k) for k in ('x','y','z')))
                    for i,key in enumerate(('x','y','z')):
                        data[key]=position[i]
                    q=target.orientation
                    desired_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    delta=(desired_yaw-data['yaw']+math.pi)%(2*math.pi)-math.pi
                    data['yaw'] += max(-.01,min(.01,delta))
                else:
                    data['velocity']=(0.,0.,0.)
                now = rospy.Time.now()
                m = pose(data['x'],data['y'],data['z'])
                m.header.stamp = now
                m.pose.orientation.z, m.pose.orientation.w = math.sin(data['yaw']/2),math.cos(data['yaw']/2)
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now,'odom','base_link'
                odom.pose.pose = m.pose
                # Odometry twist is expressed in child base_link (FLU).
                vx, vy, vz = tuple((data[k]-old[i])/.02 for i,k in enumerate(('x','y','z')))
                cy, sy = math.cos(data['yaw']), math.sin(data['yaw'])
                odom.twist.twist.linear.x = cy*vx + sy*vy
                odom.twist.twist.linear.y = -sy*vx + cy*vy
                odom.twist.twist.linear.z = vz
                fcu = State()
                fcu.header.stamp, fcu.connected, fcu.armed, fcu.mode, fcu.system_status = now,True,True,data['mode'],4
                landed = ExtendedState()
                landed.header.stamp, landed.landed_state = now,2
                rc = RCState()
                rc.header.stamp, rc.valid, rc.mode, rc.kill_switch = now,True,data['rc_mode'],False
                height = TerrainHeight()
                height.header.stamp, height.header.frame_id = now,'odom'
                height.ground_z, height.agl, height.variance, height.valid = 0.,data['z'],.001,data['terrain_valid']
                pubs['fcu'].publish(fcu); pubs['pose'].publish(m); pubs['odom'].publish(odom)
                pubs['landed'].publish(landed); pubs['rc'].publish(rc)
                pubs['external'].publish(True); pubs['height'].publish(height); pubs['terrain_ready'].publish(True)
                wall = time.monotonic()
                if wall-last_base >= .5:
                    pubs['base'].publish(True); last_base=wall
                if data['cloud'] and wall-last_cloud >= .1:
                    points=[(data['x']+x*.12,data['y']+y*.12,0.)
                            for x in range(-8,9) for y in range(-8,9)]
                    if data['obstacle']:
                        points += [(data['x']+.18, data['y']+y*.04, data['z']+z*.1)
                                   for y in range(-8,9) for z in range(-2,3)]
                    if data['pole']:
                        points += [(1.8,y*.1,z*.1) for y in range(-1,2) for z in range(0,31)]
                    pubs['cloud'].publish(point_cloud2.create_cloud_xyz32(Header(stamp=now,frame_id='odom'),points))
                    last_cloud=wall

    threading.Thread(target=sensor_loop, daemon=True).start()
    flight = launch(['roslaunch','--skip-log-check','ducted_bringup','flight_control.launch',
                     'start_rc_monitor:=false','enable_flight_output:=true'], 'flight')
    check('flight_available',wait_for(lambda:data['flight'] is not None,8.))
    rospy.wait_for_service('/ducted/control/command',timeout=3)
    control=rospy.ServiceProxy('/ducted/control/command',FlightCommand)
    time.sleep(.6)
    result=control('engage',PoseStamped())
    check('flight_engaged',result.accepted and wait_for(lambda:data['flight'].ready),result.message)
    nav=launch(['roslaunch','--skip-log-check','ducted_bringup','ego_planner.launch'], 'navigation_default')
    check('navigation_default_available',wait_for(lambda:data['navigation'] is not None,8.))
    rospy.wait_for_service('/ducted/navigation/command',timeout=3)
    navigate=rospy.ServiceProxy('/ducted/navigation/command',Navigate)
    navigate('goal','default_gate',pose(.5,0))
    time.sleep(.6)
    check('unconfirmed_hull_blocks_output',data['flight'].state=='HOLD' and abs(data['x'])<.01)
    stop(nav)
    config=yaml.safe_load((ROOT/'src/ducted_navigation/config/local_avoidance.yaml').read_text())
    # Keep the production geometry loaded by launch. Only the scene and FCU
    # are synthetic; this exercises the user's configured 60x70x20 cm body.
    config['planner']['goal_tolerance']=.06
    config['planner']['progress_timeout']=8.
    navfile=LOG/'synthetic_navigation.yaml'
    navfile.write_text(yaml.safe_dump(config))
    nav=launch(['roslaunch','--skip-log-check','ducted_bringup','ego_planner.launch',
                'enable_output:=true','geometry_confirmed:=true','config_file:='+str(navfile)], 'navigation')
    time.sleep(3.)
    navigate=rospy.ServiceProxy('/ducted/navigation/command',Navigate)
    result=navigate('goal','direct_goal',pose(.5,0))
    check('navigation_goal_accepted',result.accepted,result.message)
    check('direct_absolute_goal',wait_for(lambda:data['navigation'].state=='GOAL_REACHED'
          and data['flight'].request_id=='direct_goal' and abs(data['x']-.5)<.07,12.),
          str(data['navigation']))
    check('absolute_z_preserved',abs(data['z']-1.2)<.01)
    navigate('cancel','direct_goal',PoseStamped()); control('hold',PoseStamped())
    with lock:data['pole']=True
    time.sleep(.3)
    result=navigate('goal','pole_detour',pose(3.2,0))
    check('local detour goal accepted',result.accepted,result.message)
    check('EGO detour followed by position controller',wait_for(lambda:data['navigation'].state=='GOAL_REACHED'
          and data['flight'].request_id=='pole_detour' and abs(data['x']-3.2)<.08,45.),str(data['navigation']))
    check('detour absolute height retained',abs(data['z']-1.2)<.02)
    navigate('cancel','pole_detour',PoseStamped());control('hold',PoseStamped())
    with lock:data['pole']=False
    with lock:
        data['obstacle']=True
        blocked_x=data['x']
    result=navigate('goal','blocked_goal',pose(.8,0))
    check('obstacle_blocks_route',result.accepted and wait_for(
        lambda:data['navigation'].request_id=='blocked_goal' and data['navigation'].state=='BLOCKED'))
    time.sleep(.9)
    count=len([item for item in data['planner_targets'] if item[1]=='blocked_goal'])
    time.sleep(.25)
    check('blocked_hold_stream_is_bounded',count==len(
        [item for item in data['planner_targets'] if item[1]=='blocked_goal']))
    check('blocked_vehicle_does_not_advance',abs(data['x']-blocked_x)<.08)
    navigate('cancel','blocked_goal',PoseStamped()); control('hold',PoseStamped())
    with lock:data['obstacle']=False
    time.sleep(.8)
    mission_config=yaml.safe_load((ROOT/'src/ducted_mission/config/mission.yaml').read_text())
    mission_config['mission_id']='isolated_two_waypoints'
    mission_config['defaults'].update(xy_tolerance=.08,z_tolerance=.05,dwell=.3,timeout=25.)
    mission_config['waypoints']=[dict(frame_id='odom',position=[.7,0.,1.2],yaw=0.),
                                 dict(frame_id='odom',position=[.7,.3,1.2],yaw=0.)]
    missionfile=LOG/'synthetic_mission.yaml'
    missionfile.write_text(yaml.safe_dump(mission_config))
    args=['roslaunch','--skip-log-check','ducted_bringup','waypoint_mission.launch','mission_file:='+str(missionfile)]
    mission=launch(args,'mission_default')
    check('mission_default_available',wait_for(lambda:data['mission'] is not None,8.))
    rospy.wait_for_service('/ducted/mission/start',timeout=3)
    start=rospy.ServiceProxy('/ducted/mission/start',Trigger)
    check('mission_default_gate',not start().success)
    stop(mission)
    with lock:data['mission']=None
    mission=launch(args+['enable_commands:=true'],'mission_enabled')
    check('mission_enabled_available',wait_for(lambda:data['mission'] is not None,8.))
    time.sleep(.5)
    start=rospy.ServiceProxy('/ducted/mission/start',Trigger)
    pause=rospy.ServiceProxy('/ducted/mission/pause',Trigger)
    resume=rospy.ServiceProxy('/ducted/mission/resume',Trigger)
    cancel=rospy.ServiceProxy('/ducted/mission/cancel',Trigger)
    result=start(); check('explicit_mission_start',result.success,result.message)
    check('mission_active_id',wait_for(lambda:data['flight'].state=='TRACK'
          and data['flight'].request_id==data['mission'].request_id,5.))
    old_id=data['mission'].request_id
    result=pause(); check('mission_pause',result.success and wait_for(lambda:data['flight'].state=='HOLD'),result.message+'; '+str(data['mission']))
    time.sleep(.4)
    result=resume(); check('mission_resume',result.success,result.message)
    check('resume_new_id',wait_for(lambda:data['mission'].request_id!=old_id))
    check('two_waypoints_actual_completion',wait_for(lambda:data['mission'].state=='SUCCEEDED'
          and data['flight'].state=='HOLD',30.),str(data['mission']))
    check('last_waypoint_actual_pose',abs(data['x']-.7)<.08 and abs(data['y']-.3)<.08 and abs(data['z']-1.2)<.05)
    check('no_automatic_landing','AUTO.LAND' not in data['mode_calls'])
    result=start(); check('explicit_restart',result.success,result.message)
    check('restart_tracking',wait_for(lambda:data['flight'].state=='TRACK'))
    with lock:data['cloud']=False
    check('cloud_dropout_pauses_mission',wait_for(lambda:data['mission'].state=='PAUSED',2.))
    with lock:data['cloud']=True
    time.sleep(.6)
    result=cancel(); check('cancel_paused_mission',result.success and wait_for(lambda:data['mission'].state=='CANCELED'))
    time.sleep(.4)
    result=start(); check('restart_after_cancel',result.success,result.message)
    check('tracking_before_terrain_loss',wait_for(lambda:data['flight'].state=='TRACK'))
    with lock:data['terrain_valid']=False
    check('invalid_agl_pauses_mission',wait_for(lambda:data['mission'].state=='PAUSED',2.))
    with lock:data['terrain_valid']=True
except Exception as error:
    checks.append(dict(name='exception',passed=False,detail=repr(error)))
finally:
    done.set()
    for p in reversed(children):stop(p)
    for f in handles:f.close()
    result=dict(passed=bool(checks) and all(c['passed'] for c in checks),checks=checks,
                mode_calls=data['mode_calls'],mission_history=data['mission_history'],
                telemetry_history=data['telemetry_history'],
                planner_targets=data['planner_targets'],
                children_stopped=all(p.poll() is not None for p in children),
                scope='synthetic scene and fake FCU; no physical acceptance')
    (LOG/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('mission_history','telemetry_history','planner_targets')},indent=2),flush=True)
    if not result['passed']:raise SystemExit(1)
