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
    return dict(takeoff_rise=rise, height=height, distance=distance, hover=hover,
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


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',action='store_true')
    parser.add_argument('--confirm-ground-contact',action='store_true',
        help='confirm aircraft bottom rests on level ground, not a table or handheld')
    args=parser.parse_args()
    if args.start and args.confirm_ground_contact:
        parser.error('confirm ground contact during preparation only; --start never rebaselines')
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
        if status['state']!='IDLE' or not status.get('healthy'):
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
    command=['roslaunch','ducted_bringup','automatic_flight.launch','finish:=slow_land','takeoff_agl:=1.0',
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
