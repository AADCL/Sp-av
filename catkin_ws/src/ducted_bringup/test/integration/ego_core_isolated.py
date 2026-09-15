#!/usr/bin/env python3
"""EGO service checks on master 11325, without a map or any vehicle node."""
import sys
sys.stderr.write('EGO 已冻结 (EGO is frozen); use offline algorithm unit tests.\n')
sys.exit(2)
import os
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11325'
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ.pop('ROS_IP', None)
import json, signal, socket, subprocess, time
from pathlib import Path
import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

ROOT = Path('/home/nrc/catkin_ws')
OUT = ROOT/'logs/ego_core_isolated'
OUT.mkdir(exist_ok=True)
children, checks, requests = [], [], []

def launch(args, name):
    with (OUT/(name+'.log')).open('w') as f:
        p = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    children.append(p)

def check(name, ok, detail=''):
    checks.append(dict(name=name, passed=bool(ok), detail=detail))
    print(checks[-1], flush=True)
    if not ok:
        raise AssertionError(name)

try:
    with socket.socket() as s:
        check('isolated port unused', s.connect_ex(('127.0.0.1',11325)) != 0)
    launch(['roscore','-p','11325'], 'master')
    time.sleep(2)
    rospy.init_node('ego_core_test', disable_signals=True)
    launch(['roslaunch','--skip-log-check','ducted_bringup','ego_planner.launch'], 'ego')
    try:
        rospy.wait_for_service('/ducted/ego/plan', timeout=12)
        available = True
    except rospy.ROSException:
        available = False
    check('EGO local service available without occupancy map', available)
    from ducted_planning.srv import PlanLocal, PlanLocalRequest
    service = rospy.ServiceProxy('/ducted/ego/plan', PlanLocal)

    def request(start, goal, points, **changes):
        r = PlanLocalRequest()
        stamp_age = changes.pop('stamp_age', 0.)
        r.header = Header(stamp=rospy.Time.now()-rospy.Duration.from_sec(stamp_age), frame_id='odom')
        r.start.position.x,r.start.position.y,r.start.position.z = start
        r.goal.position.x,r.goal.position.y,r.goal.position.z = goal
        r.start.orientation.w=r.goal.orientation.w=1.
        r.clearance=.6; r.minimum_z=.9; r.maximum_z=1.9; r.sensor_horizon=4.
        r.obstacles=point_cloud2.create_cloud_xyz32(r.header,points)
        for k,v in changes.items():
            if k=='cloud_data':r.obstacles.data=v
            else:setattr(r,k,v)
        began=time.monotonic(); reply=service(r)
        requests.append(dict(success=reply.success, reason=reply.reason,
                             duration=time.monotonic()-began,
                             path=[[p.pose.position.x,p.pose.position.y,p.pose.position.z]
                                   for p in reply.local_path.poses]))
        print({k:v for k,v in requests[-1].items() if k!='path'},flush=True)
        return reply

    floor=[(x*.2,y*.2,0.) for x in range(-22,23) for y in range(-22,23)]
    direct=request((0,0,1.2),(.8,0,1.2),floor)
    check('official EGO rebound produces local spline', direct.success, direct.reason)
    check('absolute Z preserved in flat scene', all(abs(p.pose.position.z-1.2)<1e-4 for p in direct.local_path.poses))
    pole=[(0.,y*.1,z*.1) for y in range(-2,3) for z in range(0,25)]
    detour=request((-1.3,0,1.2),(1.3,0,1.2),floor+pole)
    check('EGO rebounds around local obstacle',detour.success,detour.reason)
    check('curved route deviates around obstacle', max(abs(p.pose.position.y) for p in detour.local_path.poses)>.6)
    terminal=request((1.426,.14,1.2),(1.4,0,1.2),floor)
    check('near-goal terminal chord avoids degenerate spline optimization',terminal.success,terminal.reason)
    delayed=request((0,0,1.2),(.05,0,1.2),[(3.,3.,0.)],stamp_age=.43)
    check('EGO accepts a usable snapshot within the shared 0.5 second deadline',
          delayed.success,delayed.reason)
    close_obstacle=request((1.426,.14,1.2),(1.4,0,1.2),floor+[(1.4,.07,1.2)])
    check('near-goal chord still rejects live collision',not close_obstacle.success,close_obstacle.reason)
    wall=[(0.,y*.1,z*.1) for y in range(-45,46) for z in range(0,30)]
    blocked=request((-1.3,0,1.2),(1.3,0,1.2),floor+wall)
    check('sealed local corridor rejects',not blocked.success,blocked.reason)
    bad=request((0,0,1.2),(.8,0,1.2),floor,minimum_z=1.4)
    check('measured AGL envelope rejects',not bad.success,bad.reason)
    near=request((0,0,1.2),(.8,0,1.2),floor+[(.2,0,1.2)])
    check('current swept body collision rejects',not near.success,near.reason)
    check('occupied start reports obstacle coordinates and clearance',
          all(s in near.reason for s in ('blockers=', 'nearest=', 'distance=', 'clearance=')),near.reason)
    side=request((0,0,1.2),(.12,0,1.2),floor+[(0.,-.79,1.4)],clearance=.6864952)
    check('separated side point is not blocked by isotropic voxel inflation',side.success,side.reason)
    empty=request((0,0,1.2),(.8,0,1.2),[])
    check('missing live cloud rejects',not empty.success,empty.reason)
    malformed=request((0,0,1.2),(.8,0,1.2),floor,cloud_data=b'\x00')
    check('truncated cloud rejects before PCL conversion',not malformed.success,malformed.reason)
    nonfinite=request((float('nan'),0,1.2),(.8,0,1.2),floor)
    check('nonfinite odometry rejects',not nonfinite.success,nonfinite.reason)
    huge=request((1e99,0,1.2),(.8,0,1.2),floor)
    check('unbounded coordinate rejects before voxel conversion',not huge.success,huge.reason)
    check('continuous derivative control bounds',direct.speed_bound<=.5 and direct.acceleration_bound<=.5)
    distant=request((0,0,1.2),(8,0,1.2),floor)
    check('distant goal uses bounded local horizon',distant.success and max(p.pose.position.x for p in distant.local_path.poses)<3.4,distant.reason)
    import rosgraph
    pubs=rosgraph.Master(rospy.get_name()).getSystemState()[0]
    check('no global route or vehicle publishers',not any('global_path' in t or t.startswith('/mavros') for t,n in pubs))
except Exception as error:
    checks.append(dict(name='exception',passed=False,detail=repr(error)))
finally:
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
    result=dict(passed=all(c['passed'] for c in checks),checks=checks,requests=requests)
    (OUT/'result.json').write_text(json.dumps(result,indent=2))
    print('PASS' if result['passed'] else 'FAIL',flush=True)
    raise SystemExit(0 if result['passed'] else 1)
