#!/usr/bin/env python3
"""Prepare the user's forward-flight profile; start only with explicit --start."""
import argparse
import json
import math
from pathlib import Path
import shlex
import time
import uuid


def build_profile(x, y, z, yaw, agl, height=1., distance=3., hover=5.):
    values = (x,y,z,yaw,agl,height,distance,hover)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite pose or mission input')
    if not .02 <= agl <= .6 or not .95 <= height <= 1.8 or distance <= 0 or hover < 0:
        raise ValueError('height, distance or hover time outside supported test range')
    rise = height-agl
    goal = [x+distance*math.cos(yaw), y+distance*math.sin(yaw), z+rise]
    if rise < .2 or not all(-5 <= v <= 5 for v in goal[:2]) or not -1 <= goal[2] <= 3:
        raise ValueError('target outside the existing flight envelope')
    return dict(finish='land', landing_mode='AUTO.LAND',
                takeoff_rise=rise, height=height, distance=distance, hover=hover,
                anchor=dict(x=x,y=y,z=z,yaw=yaw,agl=agl),
                waypoint=dict(frame_id='odom',position=goal,yaw=yaw,dwell=hover))


def build_contact_reference(anchor, contact_agl, run_id, stamp, confirmed):
    if confirmed is not True:
        raise ValueError('explicit confirmation of level floor contact is required')
    if (not run_id or not all(math.isfinite(float(v)) for v in (*anchor.values(),contact_agl,stamp))
            or not .02<=contact_agl<=.3 or stamp<=0):
        raise ValueError('invalid ground-contact reference')
    return dict(confirmed=True,source='operator_ground_contact',frame_id='odom',
                run_id=run_id,prepared_stamp=stamp,contact_agl=contact_agl,
                ground_z=anchor['z']-contact_agl,anchor=dict(anchor))


def build_relative_profile(x, y, z, yaw):
    if not all(math.isfinite(v) for v in (x,y,z,yaw)):
        raise ValueError('nonfinite PX4 takeoff pose')
    goal=[x+3.*math.cos(yaw),y+3.*math.sin(yaw),z+1.]
    if not -.85<=z<=2. or not all(-5<=v<=5 for v in goal[:2]):
        raise ValueError('relative takeoff and bounded landing exceed flight envelope')
    return dict(height_mode='takeoff_relative',height_source='PX4_RELATIVE',takeoff_rise=1.,
        finish='land',landing_mode='AUTO.LAND',landing_speed_source='MPC_LAND_SPEED',
        distance=3.,hover=5.,anchor=dict(x=x,y=y,z=z,yaw=yaw),
        waypoint=dict(frame_id='odom',position=goal,yaw=yaw,dwell=5.))


def relative_main(args, root, get, rospy, rospkg, yaml):
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import State, ExtendedState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from ducted_mission.ground_reference import RelativeReference
    from ducted_mission.mission import MissionConfig
    pointer=root/'logs/forward_test_latest.json'
    for name in ('/ducted/system/ready','/ducted/localization/map_ready','/ducted/external_odometry/ready'):
        if not get(name,Bool).data:raise RuntimeError(name+' is false')
    fcu=get('/mavros/state',State)
    landed=get('/mavros/extended_state',ExtendedState)
    if not fcu.connected or landed.landed_state!=1 or fcu.armed != args.start:
        raise RuntimeError('prepare disarmed on ground; --start requires operator-armed ON_GROUND')
    def sample():
        p=get('/mavros/local_position/pose',PoseStamped)
        ext=get('/mavros/odometry/out',Odometry)
        q=p.pose.orientation;v=ext.twist.twist.linear
        values=(p.pose.position.x,p.pose.position.y,p.pose.position.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z)
        if (p.header.frame_id!='odom' or ext.header.frame_id!='odom' or ext.child_frame_id!='base_link'
                or not all(math.isfinite(v) for v in values) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.001):
            raise RuntimeError('invalid PX4/external pose contract')
        delta=math.sqrt(sum((getattr(p.pose.position,k)-getattr(ext.pose.pose.position,k))**2 for k in ('x','y','z')))
        if not math.isfinite(delta) or delta>.25 or abs((p.header.stamp-ext.header.stamp).to_sec())>.15:
            raise RuntimeError('PX4/external pose mismatch or timestamps skewed')
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return dict(x=p.pose.position.x,y=p.pose.position.y,z=p.pose.position.z,yaw=yaw),math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y)))),math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
    current,tilt,speed=sample()
    if args.start:
        directory=Path(json.loads(pointer.read_text())['directory'])
        profile=json.loads((directory/'profile.json').read_text())
        if profile.get('finish') != 'land':
            raise RuntimeError('old landing profile; disarm and prepare a new AUTO.LAND task')
        reference=profile['relative_reference']
    else:
        if not args.confirm_flat_ground:
            raise RuntimeError('prepare with --confirm-flat-ground: aircraft stationary on level floor; landing area at the same level')
        reference=dict(confirmed=True,frame_id='odom',run_id=rospy.get_param('/run_id',''),
                       prepared_stamp=rospy.get_time(),anchor=current)
    datum=RelativeReference(reference,rospy.get_param('/run_id',''))
    reason=datum.check(**current,tilt=tilt,speed=speed,on_ground=True,ros_now=rospy.get_time())
    if reason:raise RuntimeError(reason)
    if args.start:
        status=json.loads(get('/ducted/automatic/status',String).data)
        if (status.get('height_mode')!='takeoff_relative' or status.get('state')!='IDLE'
                or status.get('finish')!='land'
                or not status.get('healthy') or abs(status.get('takeoff_reference_z',math.inf)-reference['anchor']['z'])>1e-6):
            raise RuntimeError('automatic stack/profile not ready: '+str(status))
        rospy.wait_for_service('/ducted/automatic/start',timeout=5.)
        response=rospy.ServiceProxy('/ducted/automatic/start',Trigger)()
        print(response)
        if not response.success:raise RuntimeError(response.message)
        return
    time.sleep(1.)
    check,tilt,speed=sample()
    if math.dist(tuple(current[k] for k in ('x','y','z')),tuple(check[k] for k in ('x','y','z')))>.03:
        raise RuntimeError('keep aircraft stationary during preparation')
    reason=datum.check(**check,tilt=tilt,speed=speed,on_ground=True,ros_now=rospy.get_time())
    if reason:raise RuntimeError(reason)
    profile=build_relative_profile(**current)
    profile['relative_reference']=reference
    rp=rospkg.RosPack()
    mission=yaml.safe_load((Path(rp.get_path('ducted_mission'))/'config/mission.yaml').read_text())
    mission['mission_id']='forward_3m_'+uuid.uuid4().hex[:8]
    mission['allowed_frames']=['odom'];mission['workspace']['z']=[-1.,3.]
    mission['waypoints']=[profile['waypoint']]
    mission['defaults'].update(xy_tolerance=.12,z_tolerance=.08,yaw_tolerance=.12,speed_tolerance=.1,dwell=5.,timeout=60.)
    MissionConfig.from_dict(mission)
    nav=yaml.safe_load((Path(rp.get_path('ducted_navigation'))/'config/local_avoidance.yaml').read_text())
    nav.update(height_mode='takeoff_relative',takeoff_reference_z=current['z'])
    nav['planner'].update(max_speed=.3,min_agl=.25,goal_tolerance=.08,progress_timeout=8.)
    directory=root/'logs'/mission['mission_id'];directory.mkdir(parents=True,exist_ok=False)
    for name,obj in (('mission',mission),('navigation',nav),('relative_reference',dict(relative_reference=reference))):
        (directory/(name+'.yaml')).write_text(yaml.safe_dump(obj))
    (directory/'profile.json').write_text(json.dumps(profile,indent=2))
    command=['roslaunch','ducted_bringup','automatic_flight.launch','height_mode:=takeoff_relative',
        'takeoff_reference_z:='+str(current['z']),'relative_reference_file:='+str(directory/'relative_reference.yaml'),
        'finish:=land','takeoff_rise:=1.0','takeoff_agl:=0.0',
        'mission_file:='+str(directory/'mission.yaml'),'navigation_config:='+str(directory/'navigation.yaml'),
        'enable_commands:=true','enable_flight_output:=true','enable_navigation_output:=true',
        'geometry_confirmed:=true','mapping_confirmed:=true']
    launcher=root/'launch_forward_test.sh'
    launcher.write_text('#!/usr/bin/env bash\nset -e\ncd /home/nrc/catkin_ws\nsource devel/setup.bash\nexec '+shlex.join(command)+'\n')
    launcher.chmod(0o755)
    pointer.write_text(json.dumps(dict(directory=str(directory))))
    print(json.dumps(profile,indent=2))
    print('Prepared only. Run bash launch_forward_test.sh; inspect status, arm manually, then run --start.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',action='store_true')
    parser.add_argument('--height-mode',choices=('takeoff_relative','terrain'),default='takeoff_relative')
    parser.add_argument('--confirm-flat-ground',action='store_true',help='confirm level, same-height takeoff and landing floor')
    parser.add_argument('--confirm-ground-contact',action='store_true',
        help='confirm aircraft bottom rests on level ground, not a table or handheld')
    args=parser.parse_args()
    if args.start and (args.confirm_ground_contact or args.confirm_flat_ground):
        parser.error('confirm the ground during preparation only; --start never rebaselines')
    import rospy
    import rospkg
    import yaml
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import State, ExtendedState
    from ducted_msgs.msg import TerrainHeight
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from ducted_mission.mission import MissionConfig
    from ducted_mission.ground_reference import TakeoffReference
    rospy.init_node('forward_test_operator',anonymous=True,disable_signals=True)
    root=Path('/home/nrc/catkin_ws')
    pointer=root/'logs/forward_test_latest.json'

    def get(topic, kind, limit=.5):
        msg=rospy.wait_for_message(topic,kind,timeout=6.)
        if hasattr(msg,'header'):
            age=(rospy.Time.now()-msg.header.stamp).to_sec()
            if not -.05 <= age <= limit:raise RuntimeError(topic+' source is stale')
        return msg

    if args.start:
        saved=Path(json.loads(pointer.read_text())['directory'])/'profile.json'
        prepared=json.loads(saved.read_text())
        if prepared.get('finish') != 'land':
            raise RuntimeError('old landing profile; disarm and prepare a new AUTO.LAND task')
        args.height_mode=prepared.get('height_mode','terrain')
    if args.height_mode=='takeoff_relative':
        if args.confirm_ground_contact:
            raise RuntimeError('new PX4-relative task uses --confirm-flat-ground; old contact mode requires --height-mode terrain')
        return relative_main(args,root,get,rospy,rospkg,yaml)

    def pose():
        msg=get('/mavros/odometry/out',Odometry)
        if (msg.header.frame_id,msg.child_frame_id)!=('odom','base_link'):
            raise RuntimeError('external odometry frame mismatch')
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
            raise RuntimeError('nonfinite external pose')
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.001:
            raise RuntimeError('invalid orientation quaternion')
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return msg,dict(x=p.x,y=p.y,z=p.z,yaw=yaw)

    # Check readiness flags; stamped telemetry and the start coordinator enforce freshness.
    for topic in ('/ducted/system/ready','/ducted/localization/map_ready',
                  '/ducted/external_odometry/ready'):
        if not get(topic,Bool).data:raise RuntimeError(topic+' is false')
    fcu=get('/mavros/state',State)
    landed=get('/mavros/extended_state',ExtendedState)
    if not fcu.connected or landed.landed_state!=1:
        raise RuntimeError('connected FCU with ON_GROUND feedback required')
    if fcu.armed != args.start:
        raise RuntimeError('prepare while disarmed; --start only after operator arming')
    odom, current=pose()
    height=get('/ducted/terrain/height',TerrainHeight)
    if height.header.frame_id!='odom':
        raise RuntimeError('terrain heartbeat must be in odom')
    if rospy.get_param('/terrain_height/agl_reference','')!='base_link':
        raise RuntimeError('terrain reference must be base_link')
    measured=(height.valid and height.header.frame_id=='odom' and math.isfinite(height.variance)
              and 0<=height.variance<=.02 and math.isfinite(height.ground_z) and math.isfinite(height.agl)
              and abs((odom.header.stamp-height.header.stamp).to_sec())<=.15
              and abs(current['z']-height.ground_z-height.agl)<=.06)
    reference=None
    if args.start:
        directory=Path(json.loads(pointer.read_text())['directory'])
        profile=json.loads((directory/'profile.json').read_text())
        reference=profile.get('ground_reference')
    elif args.confirm_ground_contact:
        geometry=rospy.get_param('/terrain_height/airframe_geometry',{})
        if (geometry.get('complete') is not True or geometry.get('frame')!='base_link'
                or geometry.get('base_link_at_horizontal_center') is not True):
            raise RuntimeError('confirmed centered aircraft geometry required')
        contact=float(geometry['base_link_height_above_bottom_m'])
        if abs(contact-float(geometry['dimensions_m']['top_to_bottom'])/2)>.001:
            raise RuntimeError('ground-contact geometry must use the center origin')
        reference=build_contact_reference(current,contact,rospy.get_param('/run_id',''),
                                           rospy.get_time(),True)
    if reference:
        q=odom.pose.pose.orientation;v=odom.twist.twist.linear
        tilt=math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y))))
        speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
        reading=TakeoffReference(reference,rospy.get_param('/run_id','')).evaluate(
            **current,tilt=tilt,speed=speed,on_ground=landed.landed_state==1,
            ros_now=rospy.get_time(),wall_now=time.monotonic(),
            measured=dict(ground_z=height.ground_z,agl=height.agl,stamp=height.header.stamp.to_sec()) if measured else None)
        if not reading.valid:raise RuntimeError(reading.reason)
        initial_agl=reading.agl
    else:
        if not measured or not get('/ducted/terrain/ready',Bool).data:
            raise RuntimeError('valid synchronized center-AGL measurement required; for level floor contact prepare with --confirm-ground-contact')
        initial_agl=height.agl
    if args.start:
        directory=Path(json.loads(pointer.read_text())['directory'])
        profile=json.loads((directory/'profile.json').read_text())
        old=profile['anchor']
        error=math.sqrt(sum((current[k]-old[k])**2 for k in ('x','y','z')))
        yaw_error=abs((current['yaw']-old['yaw']+math.pi)%(2*math.pi)-math.pi)
        if error>.08 or yaw_error>.10 or abs(initial_agl-old['agl'])>.05:
            raise RuntimeError('aircraft moved or localization changed since preparation; disarm and prepare again')
        status=json.loads(get('/ducted/automatic/status',String).data)
        if status['state']!='IDLE' or not status.get('healthy') or status.get('finish')!='land':
            raise RuntimeError('automatic node is not ready: '+str(status))
        rospy.wait_for_service('/ducted/automatic/start',timeout=5.)
        response=rospy.ServiceProxy('/ducted/automatic/start',Trigger)()
        print(response)
        if not response.success:raise RuntimeError(response.message)
        return
    time.sleep(1.)
    check_odom, check=pose()
    if math.sqrt(sum((current[k]-check[k])**2 for k in ('x','y','z')))>.03:
        raise RuntimeError('keep the aircraft stationary during preparation')
    q=check_odom.pose.pose.orientation;v=check_odom.twist.twist.linear
    if (abs((current['yaw']-check['yaw']+math.pi)%(2*math.pi)-math.pi)>.03
            or not all(math.isfinite(a) for a in (v.x,v.y,v.z))
            or math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)>.1
            or (reference and 1-2*(q.x*q.x+q.y*q.y)<math.cos(.1))):
        raise RuntimeError('keep aircraft stationary and level on the confirmed ground')
    profile=build_profile(**current,agl=initial_agl)
    profile['height_source']='CONTACT_REFERENCE' if reference else 'LIDAR'
    if reference:profile['ground_reference']=reference
    rospack=rospkg.RosPack()
    mission=yaml.safe_load((Path(rospack.get_path('ducted_mission'))/'config/mission.yaml').read_text())
    mission['mission_id']='forward_3m_'+uuid.uuid4().hex[:8]
    mission['allowed_frames']=['odom']
    mission['workspace']['z']=[-1.,3.]
    mission['waypoints']=[profile['waypoint']]
    mission['defaults'].update(xy_tolerance=.12,z_tolerance=.08,yaw_tolerance=.12,
                               speed_tolerance=.10,dwell=5.,timeout=60.)
    MissionConfig.from_dict(mission)
    nav=yaml.safe_load((Path(rospack.get_path('ducted_navigation'))/'config/local_avoidance.yaml').read_text())
    nav['planner'].update(max_speed=.3,min_agl=.3,goal_tolerance=.08,progress_timeout=8.)
    directory=root/'logs'/mission['mission_id']
    directory.mkdir(parents=True,exist_ok=False)
    (directory/'profile.json').write_text(json.dumps(profile,indent=2))
    (directory/'mission.yaml').write_text(yaml.safe_dump(mission))
    (directory/'navigation.yaml').write_text(yaml.safe_dump(nav))
    command=['roslaunch','ducted_bringup','automatic_flight.launch','finish:=land','takeoff_agl:=1.0',
             'mission_file:='+str(directory/'mission.yaml'),'navigation_config:='+str(directory/'navigation.yaml'),
             'enable_commands:=true','enable_flight_output:=true','enable_navigation_output:=true',
             'geometry_confirmed:=true','mapping_confirmed:=true']
    if reference:
        reference_file=directory/'ground_reference.yaml'
        reference_file.write_text(yaml.safe_dump(dict(ground_reference=reference)))
        command.append('ground_reference_file:='+str(reference_file))
    launcher=root/'launch_forward_test.sh'
    launcher.write_text('#!/usr/bin/env bash\nset -e\ncd /home/nrc/catkin_ws\nsource devel/setup.bash\nexec '+shlex.join(command)+'\n')
    launcher.chmod(0o755)
    pointer.write_text(json.dumps(dict(directory=str(directory))))
    print(json.dumps(profile,indent=2))
    print('Prepared only. Start nodes: ./launch_forward_test.sh')
    print('After operator arming, start mission: python3 prepare_forward_test.py --start')


if __name__=='__main__':
    main()
