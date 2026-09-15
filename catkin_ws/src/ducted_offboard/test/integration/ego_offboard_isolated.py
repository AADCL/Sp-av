#!/usr/bin/env python3
"""Production EGO + navigation + C++ OFFBOARD against isolated fake MAVROS."""
import os
import sys

if __name__ == '__main__':
    sys.stderr.write('EGO 已冻结 (EGO is frozen); use offline algorithm unit tests.\n')
    sys.exit(2)
import json
import math
import socket
import threading
import time
from pathlib import Path
from xmlrpc.client import ServerProxy
import offboard_isolated as base

PORT = 11328
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % PORT
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ.pop('ROS_IP', None)
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import ExtendedState
from std_msgs.msg import Header, Bool
from ducted_msgs.msg import FlightSetpoint
from ducted_navigation.msg import NavigationStatus
from ducted_navigation.srv import Navigate

OUT = base.ROOT/'logs'/'ego_offboard_isolated'
base.LOG_ROOT = OUT
checks, processes = [], []


def check(name, condition=True):
    if not condition:
        raise AssertionError(name)
    checks.append(name)
    print('PASS '+name, flush=True)


class Scene(base.FakeMavros):
    def __init__(self):
        self.cloud_enabled = True
        self.external_offset = 0.
        self.scene = 'pole'
        self.last_cloud = 0.
        self.previous_position = None
        self.track = []
        self.odom_pub = rospy.Publisher('/mavros/odometry/out',Odometry,queue_size=3)
        self.cloud_pub = rospy.Publisher('/ducted/localization/cloud_registered',PointCloud2,queue_size=1)
        self.base_pub = rospy.Publisher('/ducted/system/ready',Bool,queue_size=1)
        self.external_pub = rospy.Publisher('/ducted/external_odometry/ready',Bool,queue_size=1)
        super().__init__()
        with self.lock:
            self.position = [-1.4,0.,0.]

    def advance_offboard(self):
        # A bounded first-order position servo. Unlike instantaneous teleporting
        # to each 20 Hz setpoint, it provides physically consistent 50 Hz speed.
        delta=[self.target[i]-self.position[i] for i in range(3)]
        desired=[4*v for v in delta]
        horizontal=math.hypot(*desired[:2])
        if horizontal>.3:desired[:2]=[v*.3/horizontal for v in desired[:2]]
        desired[2]=max(-.2,min(.2,desired[2]))
        velocity=getattr(self,'sim_velocity',[0.,0.,0.])
        dv=[desired[i]-velocity[i] for i in range(3)]
        norm=math.sqrt(sum(v*v for v in dv))
        scale=min(1.,.5*.02/norm) if norm else 1.
        self.sim_velocity=[velocity[i]+dv[i]*scale for i in range(3)]
        self.position=[self.position[i]+self.sim_velocity[i]*.02 for i in range(3)]
        yaw_delta=(self.target[3]-self.yaw+math.pi)%(2*math.pi)-math.pi
        self.yaw+=max(-.007,min(.007,yaw_delta))
        if self.position[2]>.05:self.landed=ExtendedState.LANDED_STATE_IN_AIR

    def publish(self,event):
        super().publish(event)
        with self.lock:
            position=list(self.position);yaw=self.yaw;pose_enabled=self.publish_pose
            velocity=[0.,0.,0.] if self.previous_position is None else [
                (position[i]-self.previous_position[i])/.02 for i in range(3)]
            self.previous_position=position
        stamp=self.last_pose_stamp  # Same physical pose sample as fake FCU.
        odom=Odometry(header=Header(stamp=stamp,frame_id='odom'),child_frame_id='base_link')
        odom.pose.pose.position.x=position[0]+self.external_offset
        odom.pose.pose.position.y=position[1];odom.pose.pose.position.z=position[2]
        odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
        # nav_msgs/Odometry twist is expressed in child_frame_id (base_link).
        odom.twist.twist.linear.x=math.cos(yaw)*velocity[0]+math.sin(yaw)*velocity[1]
        odom.twist.twist.linear.y=-math.sin(yaw)*velocity[0]+math.cos(yaw)*velocity[1]
        odom.twist.twist.linear.z=velocity[2]
        if pose_enabled:self.odom_pub.publish(odom)
        self.base_pub.publish(Bool(True));self.external_pub.publish(Bool(True))
        self.track.append((time.monotonic(),*position))
        now=time.monotonic()
        if self.cloud_enabled and now-self.last_cloud>=.099:
            self.last_cloud=now
            floor=[(x*.25,y*.25,-.1) for x in range(-25,26) for y in range(-20,21)]
            obstacles=[]
            if self.scene=='pole':obstacles=[(0.,y*.1,z*.1) for y in range(-1,2) for z in range(0,25)]
            elif self.scene=='blocked':obstacles=[(position[0]+.05,position[1],position[2])]
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(odom.header,floor+obstacles))


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('isolated test port already in use')
    master=base.launch(['roscore','-p',str(PORT)],'master');processes.append(master)
    fake=None
    try:
        base.wait_for(lambda:ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0]==1,10,'master')
        rospy.init_node('ego_offboard_test',disable_signals=True)
        rospy.set_param('/ducted_navigation/failure_log_dir',str(OUT/'failures'))
        fake=Scene();sink=base.StatusSink()
        sequence_pub=rospy.Publisher('/ctrl_cmd/waypoints',NavPath,queue_size=1)
        stray_pub=rospy.Publisher('/ducted/control/target',FlightSetpoint,queue_size=1)
        paths=[];nav_status=[];targets=[]
        rospy.Subscriber('/ducted/control/target',FlightSetpoint,lambda m:targets.append((time.monotonic(),rospy.Time.now().to_sec()-m.header.stamp.to_sec(),m.request_id)),queue_size=20)
        path_sub=rospy.Subscriber('/ducted/ego/local_path',NavPath,lambda m:paths.append(m),queue_size=1)
        rospy.Subscriber('/ducted/navigation/status',NavigationStatus,lambda m:nav_status.append((time.monotonic(),m.state,m.reason)),queue_size=20)
        navigation=base.launch(['roslaunch','--skip-log-check','ducted_bringup','local_avoidance.launch',
            'require_planner_context:=true','height_mode:=takeoff_relative',
            'geometry_confirmed:=true','enable_output:=true','horizontal_speed:=0.3',
            'vertical_speed:=0.2','snapshot_motion_margin:=0.10','controller_frame_margin:=0.05'],'navigation')
        processes.append(navigation)
        rospy.wait_for_service('/ducted/navigation/command',timeout=15)
        delay=[0.]
        proxy=rospy.ServiceProxy('/ducted/navigation/command',Navigate)
        def forward(request):
            duration=delay[0]
            if duration:time.sleep(duration)
            return proxy(request)
        service=rospy.Service('/test/navigation_command',Navigate,forward)
        rospy.set_param('/ctrl_cmd/state',0)
        def command(value):
            rospy.set_param('/ctrl_cmd/state',value)
            return base.wait_for(lambda:sink.latest and sink.latest.command==value,3,'command '+str(value))
        controller=base.launch(base.node_command(execution_mode='ego',takeoff_height=1,
            horizontal_speed=.3,vertical_speed=.2,yaw_rate_deg=20,
            arrival_dwell=.2,waypoint_timeout=100,takeoff_timeout=15,
            planner_command_service='/test/navigation_command',planner_service_timeout=1,
            log_root=str(OUT/'sessions')),'controller')
        processes.append(controller)
        base.wait_for(lambda:sink.phase('IDLE_GROUND') and sequence_pub.get_num_connections()>0,10,'idle controller')
        base.publish_sequence(sequence_pub,[(2.8,0,1,0),(2.8,.35,1,0)])
        base.wait_for(lambda:sink.latest and sink.latest.waypoint_count==2,3,'mission loaded')
        command(1)
        first=base.wait_for(lambda:sink.phase('HOLDING') or sink.phase('PAUSED') or sink.phase('ERROR'),18,'takeoff')
        check('takeoff holds without granting EGO execution',first.phase=='HOLDING' and
              not first.planner_ready and not first.planner_request_id)
        hold_position=tuple(fake.position)
        time.sleep(.6)
        check('loaded mission waits without EGO path or horizontal motion',bool(sink.phase('HOLDING')) and
              not paths and not targets and math.dist(hold_position,tuple(fake.position))<.08)
        command(2)
        first=base.wait_for(lambda:sink.phase('GUIDING') or sink.phase('PAUSED') or sink.phase('ERROR'),3,'explicit guiding')
        check('state=2 submits first EGO goal',first.phase=='GUIDING' and bool(first.planner_request_id))
        base.wait_for(lambda:paths or sink.phase('PAUSED'),8,'first real EGO path')
        path_sub.unregister()  # Do not accumulate large visualization Paths in the fake FCU process.
        check('real EGO emits spline',bool(paths))
        check('curve bends around pole',any(max(abs(p.pose.position.y) for p in path.poses)>.5 for path in paths))
        # A blocked/stale pipeline is a failure, not successful waypoint completion.
        result=base.wait_for(lambda:sink.phase('HOLDING') or sink.phase('PAUSED') or sink.phase('ERROR'),100,'two waypoint mission')
        check('two waypoints complete without direct fallback',result.phase=='HOLDING' and result.current_waypoint==2)
        check('executed trajectory goes around pole',max(abs(p[2]) for p in fake.track)>.5)
        check('only C++ controller publishes PX4 setpoints',
              dict(ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/probe')[2][0]).get(
                  '/mavros/setpoint_position/local')==['/ducted_offboard_controller'])
        # Pause freezes execution; a replacement sequence resumes from point one.
        command(0);base.wait_for(lambda:sink.phase('PAUSED'),3,'pause from hold')
        fake.scene='clear'
        base.publish_sequence(sequence_pub,[(2.5,.35,1,0)])
        time.sleep(.25)
        check('replacement while paused never auto resumes',bool(sink.phase('PAUSED')))
        command(2)
        resumed=base.wait_for(lambda:sink.phase('GUIDING'),3,'resume')
        old_id=resumed.planner_request_id
        base.wait_for(lambda:sink.latest and sink.latest.planner_target_age>=0,4,'resumed target')
        fake.cloud_enabled=False
        paused=base.wait_for(lambda:sink.phase('PAUSED'),3,'cloud loss pause')
        check('cloud loss pauses and revokes planner',not paused.planner_ready)
        bad=FlightSetpoint(header=Header(stamp=rospy.Time.now(),frame_id='odom'),request_id=old_id)
        bad.pose.position.x=99;bad.pose.position.z=1;bad.pose.orientation.w=1
        stray_pub.publish(bad);fake.cloud_enabled=True;time.sleep(.4)
        check('old task result and recovered cloud do not resume',bool(sink.phase('PAUSED')) and abs(fake.target[0])<10)
        command(0);time.sleep(.15)
        delay[0]=2.0
        before=len(fake.setpoint_times)
        command(2)
        paused=base.wait_for(lambda:sink.phase('PAUSED'),4,'service timeout')
        check('blocked navigation service does not block setpoint loop',len(fake.setpoint_times)-before>=15)
        check('navigation service timeout pauses', 'service timeout' in paused.reason)
        time.sleep(1.3);delay[0]=0
        check('late navigation service response cannot resume',bool(sink.phase('PAUSED')))
        command(0);time.sleep(.15)
        fake.scene='blocked'
        command(2)
        paused=base.wait_for(lambda:sink.phase('PAUSED'),4,'blocked scene')
        check('real EGO rejects obstructed scene',bool(paused))
        fake.scene='clear'
        command(0);time.sleep(.15)
        fake.external_offset=.3
        command(2)
        paused=base.wait_for(lambda:sink.phase('PAUSED'),3,'frame mismatch')
        check('external odometry divergence pauses','aligned' in paused.reason)
        fake.external_offset=0
        command(3)
        base.wait_for(lambda:sink.phase('IDLE_GROUND'),12,'land and disarm')
        check('AUTO.LAND and normal disarm complete',not fake.armed and any(not v for _,v in fake.arm_calls))
        # Same node, new takeoff datum. A new floor-level simulation is deliberate.
        with fake.lock:fake.position=[2.,0.,.2];fake.target=None
        command(0);time.sleep(.3)
        base.publish_sequence(sequence_pub,[(.4,0,1,0)])
        command(1)
        second=base.wait_for(lambda:sink.phase('HOLDING') or sink.phase('ERROR'),18,'second round hold')
        check('second takeoff also waits for state=2',second.phase=='HOLDING' and
              not second.planner_ready and not second.planner_request_id)
        command(2)
        second=base.wait_for(lambda:sink.phase('GUIDING') or sink.phase('ERROR'),3,'second round guiding')
        check('second round updates datum and task ID',abs(second.takeoff_reference_z-.2)<.01 and second.planner_request_id!=old_id)
        # Manual mode exit remains owned by the operator.
        with fake.lock:fake.mode='POSCTL';fake.pending_mode=None
        base.wait_for(lambda:sink.phase('ERROR'),3,'manual takeover')
        count=len(fake.setpoint_times);time.sleep(.3)
        check('manual mode exit stops setpoints',len(fake.setpoint_times)==count)
        return True
    finally:
        if fake is not None:fake.timer.shutdown()
        for p in reversed(processes):base.stop(p)
        if fake is not None:(OUT/'track.json').write_text(json.dumps(fake.track))
        if 'targets' in locals():(OUT/'targets.json').write_text(json.dumps(targets))
        if 'nav_status' in locals():(OUT/'navigation_status.json').write_text(json.dumps(nav_status))


if __name__=='__main__':
    passed=False;error=''
    try:passed=main()
    except Exception as exc:error=repr(exc);print(error,flush=True)
    OUT.mkdir(exist_ok=True,parents=True)
    (OUT/'result.json').write_text(json.dumps(dict(passed=passed,checks=checks,error=error,
        scope='isolated fake MAVROS, production EGO/navigation/OFFBOARD; no hardware'),indent=2))
    sys.exit(0 if passed else 1)
