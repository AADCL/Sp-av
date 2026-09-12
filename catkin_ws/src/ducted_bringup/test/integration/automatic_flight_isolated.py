#!/usr/bin/env python3
"""Connect real planner, mission and flight nodes to an isolated fake vehicle."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import subprocess
import threading
import time
from xmlrpc.client import ServerProxy
import yaml

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11327'
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
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger

ROOT = Path('/home/nrc/catkin_ws')
NO_ACQUISITION = '--ground-contact-no-acquisition' in sys.argv
GROUND_CONTACT = '--ground-contact' in sys.argv or NO_ACQUISITION
SLOW_FORWARD = '--slow-forward' in sys.argv or GROUND_CONTACT
LOG = ROOT/('logs/ground_contact_no_acquisition' if NO_ACQUISITION else
            'logs/ground_contact_isolated' if GROUND_CONTACT else
            'logs/slow_forward_isolated' if SLOW_FORWARD else 'logs/automatic_flight_isolated')
LOG.mkdir(exist_ok=True)
children, handles, checks = [], [], []
done = threading.Event()
lock = threading.RLock()
data = dict(x=0., y=0., z=.4, yaw=0., mode='POSCTL', rc_mode='command',
            target=None, flight=None, navigation=None, mission=None, cloud=True,
            obstacle=False, follow=True, armed=False, automatic=None, automatic_history=[], mode_calls=[], mission_history=[], telemetry_history=[],
            planner_targets=[], terrain_valid=True, velocity=(0., 0., 0.))
data['slow_targets'] = []
data['land_entry_z'] = None
data['height_sources'] = []
if SLOW_FORWARD: data['yaw'] = math.pi/3

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
            if key=='automatic':
                data['automatic']=json.loads(m.data);data['automatic_history'].append(data['automatic']);return
            data[key] = m
            if key=='flight':data['height_sources'].append((m.state,m.height_source,m.height_reference_valid))
            if key == 'target' and data['flight'] is not None and data['flight'].state == 'SLOW_DESCENT':
                data['slow_targets'].append((m.header.stamp.to_sec(),m.pose.position.z))
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
        if request.custom_mode == 'AUTO.LAND': data['land_entry_z'] = data['z']
    return SetModeResponse(mode_sent=True)

def pose(x, y, z=1.2):
    m = PoseStamped()
    m.header.stamp, m.header.frame_id = rospy.Time.now(), 'odom'
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x,y,z
    m.pose.orientation.w = 1.
    return m

try:
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 11327)) == 0:
            raise RuntimeError('test port11327 already in use; refusing to share a master')
    launch(['roscore', '-p', '11327'], 'master')
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
    rospy.init_node('automatic_flight_harness', disable_signals=True)
    rospy.set_param('/terrain_height/agl_reference', 'base_link')
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
            rospy.Subscriber('/ducted/mission/status', MissionStatus, receive('mission'), queue_size=20),
            rospy.Subscriber('/ducted/automatic/status', String, receive('automatic'), queue_size=20)]

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
                    data['z'] = max(.4, data['z'])
                    q=target.orientation
                    desired_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    delta=(desired_yaw-data['yaw']+math.pi)%(2*math.pi)-math.pi
                    data['yaw'] += max(-.01,min(.01,delta))
                elif data['mode']=='AUTO.LAND':
                    data['z']=max(.4,data['z']-.006)
                    if data['z']<=.401:data['armed']=False
                    data['velocity']=(0.,0.,-.3 if data['armed'] else 0.)
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
                fcu.header.stamp, fcu.connected, fcu.armed, fcu.mode, fcu.system_status = now,True,data['armed'],data['mode'],4
                landed = ExtendedState()
                landed.header.stamp, landed.landed_state = now,(1 if data['z']<=.41 else 2)
                rc = RCState()
                rc.header.stamp, rc.valid, rc.mode, rc.kill_switch = now,True,data['rc_mode'],False
                height = TerrainHeight()
                height.header.stamp, height.header.frame_id = now,'odom'
                height.ground_z, height.agl, height.variance, height.valid = .3,data['z']-.3,.001,data['terrain_valid']
                if GROUND_CONTACT:
                    height.valid=(not NO_ACQUISITION and height.agl>.35)
                    if not height.valid:
                        height.ground_z,height.agl,height.variance=math.nan,math.nan,math.inf
                        height.reason='insufficient terrain support at reference'
                pubs['fcu'].publish(fcu); pubs['pose'].publish(m); pubs['odom'].publish(odom)
                pubs['landed'].publish(landed); pubs['rc'].publish(rc)
                pubs['external'].publish(True); pubs['height'].publish(height); pubs['terrain_ready'].publish(height.valid)
                wall = time.monotonic()
                if wall-last_base >= .5:
                    pubs['base'].publish(True); last_base=wall
                if data['cloud'] and wall-last_cloud >= .1:
                    points=[(data['x']+x*.12,data['y']+y*.12,.3)
                            for x in range(-8,9) for y in range(-8,9)]
                    if data['obstacle']:
                        points += [(data['x']+.18, data['y']+y*.04, data['z']+z*.1)
                                   for y in range(-8,9) for z in range(-2,3)]
                    pubs['cloud'].publish(point_cloud2.create_cloud_xyz32(Header(stamp=now,frame_id='odom'),points))
                    last_cloud=wall

    threading.Thread(target=sensor_loop, daemon=True).start()
    mission_config=yaml.safe_load((ROOT/'src/ducted_mission/config/mission.yaml').read_text())
    mission_config['mission_id']='automatic_two_waypoints'
    mission_config['defaults'].update(xy_tolerance=.08,z_tolerance=.05,dwell=.25,timeout=25.)
    mission_config['waypoints']=[dict(frame_id='odom',position=[.4,0.,1.5],yaw=0.),
                                 dict(frame_id='odom',position=[.4,.3,1.5],yaw=0.)]
    if SLOW_FORWARD:
        mission_config['defaults'].update(dwell=5.,timeout=60.)
        mission_config['waypoints']=[dict(frame_id='odom',position=[1.5,3*math.sin(math.pi/3),1.3],yaw=math.pi/3)]
    missionfile=LOG/'mission.yaml';missionfile.write_text(yaml.safe_dump(mission_config))
    config=yaml.safe_load((ROOT/'src/ducted_navigation/config/local_avoidance.yaml').read_text())
    config['planner']['goal_tolerance']=.06;config['planner']['progress_timeout']=8.
    if SLOW_FORWARD: config['planner'].update(min_agl=.3,max_speed=.3)
    navfile=LOG/'navigation.yaml';navfile.write_text(yaml.safe_dump(config))
    args=['roslaunch','--skip-log-check','ducted_bringup','automatic_flight.launch',
          'start_rc_monitor:=false','finish:=land','mission_file:='+str(missionfile),
          'navigation_config:='+str(navfile)]
    if SLOW_FORWARD:
        args=[v for v in args if v!='finish:=land']+['finish:=slow_land','takeoff_agl:=1.0']
    if GROUND_CONTACT:
        from ducted_mission.ground_reference import TakeoffReference
        reference=dict(confirmed=True,source='operator_ground_contact',frame_id='odom',
            run_id=rospy.get_param('/run_id'),prepared_stamp=rospy.get_time(),contact_agl=.1,
            ground_z=.3,anchor=dict(x=0.,y=0.,z=.4,yaw=data['yaw']))
        ref_file=LOG/'ground_reference.yaml'
        ref_file.write_text(yaml.safe_dump(dict(ground_reference=reference)))
        args+=['ground_reference_file:='+str(ref_file)]
    stack=launch(args,'automatic_default')
    check('default automatic node available',wait_for(lambda:data['automatic'] is not None,15.))
    start=rospy.ServiceProxy('/ducted/automatic/start',Trigger)
    cancel=rospy.ServiceProxy('/ducted/automatic/cancel',Trigger)
    check('default automatic gate rejects',not start().success)
    check('launch makes no mode changes',not data['mode_calls'])
    stop(stack)
    with lock:
        data['automatic']=None;data['armed']=True
    partial=launch(args+['enable_commands:=true','enable_flight_output:=true',
                         'enable_navigation_output:=true'],'automatic_geometry_closed')
    check('partially enabled stack available',wait_for(lambda:data['automatic'] is not None,15.))
    time.sleep(.6)
    start=rospy.ServiceProxy('/ducted/automatic/start',Trigger)
    result=start()
    check('geometry gate prevents takeoff before navigation',not result.success and 'geometry_confirmed' in result.message,result.message)
    check('partial configuration sends no mode command',not data['mode_calls'])
    stop(partial)
    with lock:
        data['automatic']=None;data['flight']=None;data['mission']=None;data['navigation']=None;data['armed']=False
    stack=launch(args+['enable_commands:=true','enable_flight_output:=true',
                       'enable_navigation_output:=true','geometry_confirmed:=true'],'automatic_enabled')
    check('enabled stack available',wait_for(lambda:all(data[k] is not None for k in ('automatic','flight','mission','navigation')),16.))
    start=rospy.ServiceProxy('/ducted/automatic/start',Trigger)
    cancel=rospy.ServiceProxy('/ducted/automatic/cancel',Trigger)
    time.sleep(.7)
    check('unarmed aircraft rejects automatic start',not start().success)
    with lock:data['armed']=True;data['obstacle']=True
    time.sleep(.3)
    check('occupied takeoff corridor rejects',not start().success)
    with lock:data['obstacle']=False
    time.sleep(.8)
    result=start();check('explicit automatic start accepted',result.success,result.message)
    check('takeoff observed',wait_for(lambda:data['flight'].state=='TAKEOFF',10.),str(data['automatic']))
    if NO_ACQUISITION:
        check('missing height acquisition aborts vertical initialization',
              wait_for(lambda:data['automatic']['state'] in ('ABORTED','FAULT'),12.),str(data['automatic']))
        check('no waypoint target without measured ground',not data['planner_targets'])
        check('initialization height remains bounded',data['z']<=1.41,data['z'])
        check('no automatic landing or restart on acquisition failure','AUTO.LAND' not in data['mode_calls'])
        check('healthy flight controller holds after height failure',wait_for(lambda:data['flight'].state=='HOLD',3.))
        raise SystemExit(0)
    check('waypoint tracking after measured takeoff',wait_for(lambda:data['flight'].state=='TRACK',20.),str(data['automatic']))
    check('automatic waypoint and landing sequence completed',wait_for(lambda:data['automatic']['state']=='SUCCEEDED',65. if SLOW_FORWARD else 45.),str(data['automatic']))
    check('completion requires ground and disarmed',data['z']<=.41 and not data['armed'])
    check('OFFBOARD and landing each requested once',data['mode_calls'].count('OFFBOARD')==1 and data['mode_calls'].count('AUTO.LAND')==1,str(data['mode_calls']))
    expected=(1.5,3*math.sin(math.pi/3)) if SLOW_FORWARD else (.4,.3)
    check('actual waypoint reached before landing',any(s.get('state')=='LANDING' for s in data['automatic_history']) and abs(data['x']-expected[0])<.08 and abs(data['y']-expected[1])<.08)
    if SLOW_FORWARD:
        samples=data['slow_targets']
        check('controlled descent setpoints observed',len(samples)>20,len(samples))
        rates=[(a[1]-b[1])/(b[0]-a[0]) for a,b in zip(samples,samples[1:]) if b[0]>a[0]]
        check('descent target rate limited to 0.2 m/s',max(rates)<=.205,max(rates))
        check('AUTO.LAND waits for measured contact',data['land_entry_z']<=.41,data['land_entry_z'])
    if GROUND_CONTACT:
        check('contact datum used before lidar acquisition',any(s.get('height_source')=='CONTACT_REFERENCE' for s in data['automatic_history']))
        check('takeoff handed over to measured lidar height',any(s.get('height_measured') and s.get('state')=='RUNNING' for s in data['automatic_history']))
        check('near-ground descent used explicit landing reference',any(source=='LANDING_REFERENCE' and valid for _,source,valid in data['height_sources']))
    count=len(data['mode_calls']);time.sleep(.6)
    check('completed flight does not restart itself',len(data['mode_calls'])==count)
    if SLOW_FORWARD:
        raise SystemExit(0)  # This mode only verifies the requested complete flight.
    # A second explicit run is stopped during prestream; it must never take off.
    with lock:data['mode']='POSCTL';data['armed']=True;data['target']=None
    time.sleep(.6)
    result=start();check('second explicit ground start accepted',result.success,result.message)
    result=cancel();check('cancel accepted',result.success)
    check('canceled sequence stops',wait_for(lambda:data['automatic']['state'] in ('ABORTED','FAULT'),8.),str(data['automatic']))
    time.sleep(1.4)
    check('canceled prestream cannot start later',data['z']<=.41 and data['mode_calls'].count('OFFBOARD')==1)
    result=start();check('third explicit ground start accepted',result.success,result.message)
    check('tracking before coordinator interruption',wait_for(lambda:data['flight'].state=='TRACK',25.),str(data['automatic']))
    # Freeze only the fake-master coordinator. Its independent consumers must
    # expire the lease even if the coordinator cannot run its cancellation code.
    import rosgraph
    master=rosgraph.Master(rospy.get_name())
    uri=master.lookupNode('/automatic_flight')
    with ServerProxy(uri) as api:
        code,message,automatic_pid=api.getPid(rospy.get_name())
    check('isolated coordinator PID resolved',code==1 and automatic_pid>1)
    os.kill(automatic_pid,signal.SIGSTOP)
    try:
        check('flight lease loss revokes output',wait_for(lambda:data['flight'].state=='DISABLED',2.))
        check('mission lease loss pauses task',wait_for(lambda:data['mission'].state=='PAUSED',2.))
        stamp=data['target'].header.stamp.to_sec();time.sleep(.4)
        check('no flight setpoints after lease expiry',data['target'].header.stamp.to_sec()==stamp)
    finally:
        os.kill(automatic_pid,signal.SIGCONT)
    check('recovered coordinator does not resume mission',wait_for(lambda:data['automatic']['state'] in ('ABORTED','FAULT'),5.))
except Exception as error:
    checks.append(dict(name='exception',passed=False,detail=repr(error)))
finally:
    done.set()
    for p in reversed(children):stop(p)
    for f in handles:f.close()
    result=dict(passed=bool(checks) and all(c['passed'] for c in checks),checks=checks,
                mode_calls=data['mode_calls'],automatic_history=data['automatic_history'],mission_history=data['mission_history'],
                telemetry_history=data['telemetry_history'],
                planner_targets=data['planner_targets'],height_sources=data['height_sources'],
                children_stopped=all(p.poll() is not None for p in children),
                scope='synthetic scene and fake FCU; no physical acceptance')
    (LOG/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('mission_history','telemetry_history','planner_targets')},indent=2),flush=True)
    if not result['passed']:raise SystemExit(1)
