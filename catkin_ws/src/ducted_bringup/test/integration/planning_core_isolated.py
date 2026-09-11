#!/usr/bin/env python3
"""Real Fast-Planner service on an isolated master; no vehicle nodes."""
import os
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11325'
os.environ['ROS_HOSTNAME']='127.0.0.1'
os.environ.pop('ROS_IP',None)
import json,signal,socket,subprocess,time
from pathlib import Path
import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from ducted_planning.srv import PlanPath,PlanPathRequest

root=Path('/home/nrc/catkin_ws');out=root/'logs/planning_core_isolated';out.mkdir(exist_ok=True)
children=[];checks=[];details=[]
def launch(args,name):
    with (out/(name+'.log')).open('w') as f:
        p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    children.append(p);return p
def check(name,passed,detail=''):
    checks.append(dict(name=name,passed=bool(passed),detail=detail));print(checks[-1],flush=True)
    if not passed:raise AssertionError(name)
def request(start,goal,points,**changes):
    r=PlanPathRequest();r.header=Header(stamp=rospy.Time.now(),frame_id='odom')
    r.start.position.x,r.start.position.y,r.start.position.z=start
    r.goal.position.x,r.goal.position.y,r.goal.position.z=goal
    r.start.orientation.w=r.goal.orientation.w=r.map_to_odom.rotation.w=1.
    r.clearance=.6;r.minimum_z=.95;r.maximum_z=1.95
    r.obstacles=point_cloud2.create_cloud_xyz32(r.header,points)
    for k,v in changes.items():setattr(r,k,v)
    begin=time.monotonic();response=service(r);duration=time.monotonic()-begin
    details.append(dict(start=start,goal=goal,success=response.success,reason=response.reason,duration=duration,
                        global_path=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in response.global_path.poses]))
    print(details[-1],flush=True);return response
try:
    with socket.socket() as s:check('port unused',s.connect_ex(('127.0.0.1',11325))!=0)
    launch(['roscore','-p','11325'],'master');time.sleep(2)
    rospy.init_node('planning_core_test',disable_signals=True)
    points=[((x+.5)*.2,(y+.5)*.2,(z+.5)*.2,-3.5) for x in range(-25,25) for y in range(-25,25) for z in range(-5,20)]
    file=out/'observed_occupancy.pcd'
    with file.open('w') as f:
        f.write('VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA ascii\n'%(len(points),len(points)))
        for p in points:f.write('%f %f %f %f\n'%p)
    (out/'mapping_metadata.yaml').write_text('format_version: 1\nframe_id: map\noccupancy_resolution: 0.2\n')
    import yaml
    for k,v in yaml.safe_load((root/'src/ducted_planning/config/planning.yaml').read_text()).items():rospy.set_param('/core/'+k,v)
    rospy.set_param('/core/occupancy_file',str(file))
    launch(['rosrun','ducted_planning','global_local_planner','__name:=core'],'core_configured')
    rospy.wait_for_service('/core/plan',timeout=15);service=rospy.ServiceProxy('/core/plan',PlanPath)
    floor=[(x*.2,y*.2,0.) for x in range(-24,25) for y in range(-24,25)]
    # Warm transformed map cache without accepting an expired first result.
    first=request((0,0,1.2),(.7,0,1.2),floor)
    direct=request((0,0,1.2),(.7,0,1.2),floor)
    check('direct kino and spline trajectory',direct.success,direct.reason)
    check('absolute goal z retained',all(abs(p.pose.position.z-1.2)<1e-4 for p in direct.local_path.poses))
    wall=[(0.,y*.2,z*.2) for y in range(-24,9) for z in range(-5,20)]
    detour=request((-1.5,0,1.2),(1.5,0,1.2),floor+wall)
    check('global wall detour with local trajectory',detour.success and len(detour.global_path.poses)>2,detour.reason)
    full_wall=[(0.,y*.2,z*.2) for y in range(-25,26) for z in range(-5,21)]
    blocked=request((-1.5,0,1.2),(1.5,0,1.2),floor+full_wall)
    check('fully blocked map rejects',not blocked.success,blocked.reason)
    unknown=request((0,0,1.2),(5.5,0,1.2),floor)
    check('unknown target rejects',not unknown.success,unknown.reason)
    bad_agl=request((0,0,1.2),(.7,0,1.2),floor,minimum_z=1.5)
    check('measured AGL envelope rejects',not bad_agl.success,bad_agl.reason)
    person=request((0,0,1.2),(.7,0,1.2),floor+[(.2,0,1.2)])
    check('live dynamic obstacle retained',not person.success,person.reason)
    import rosgraph
    pubs=rosgraph.Master(rospy.get_name()).getSystemState()[0]
    check('no vehicle command publishers',not any(t.startswith('/mavros') or t=='/ducted/control/target' for t,n in pubs))
except Exception as e:
    checks.append(dict(name='exception',passed=False,detail=repr(e)))
finally:
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
    result=dict(passed=all(c['passed'] for c in checks),checks=checks,requests=details)
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
    raise SystemExit(0 if result['passed'] else 1)
