#!/usr/bin/env python3
"""Coordinate production flight and waypoint nodes after an explicit start."""
from dataclasses import fields
from collections import deque
import json
import math
import threading
from time import monotonic
import rospy
import tf2_ros
from ducted_mission.automatic import AutomaticFlight, AutoConfig, Observation, matched_cloud_pose
from ducted_mission.automatic_services import bootstrap
from ducted_mission.runtime import PersistentDeadlineProcess
from ducted_mission.mission import StampedInput
from ducted_mission.msg import MissionStatus
from ducted_msgs.msg import RCState, FlightControlStatus, TerrainHeight
from ducted_navigation.msg import NavigationStatus
from ducted_navigation.runtime import transform_points
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


class AutomaticNode:
    def __init__(self):
        values={f.name:rospy.get_param('~'+f.name,f.default) for f in fields(AutoConfig)}
        values['enabled']=rospy.get_param('~enable_commands',False) is True
        self.core=AutomaticFlight(AutoConfig(**values));self.lock=threading.RLock()
        required_gates=(
            '/flight_controller/flight/enable_flight_output',
            '/flight_controller/flight/require_automatic_lease',
            '/ducted_navigation/enable_output', '/ducted_navigation/geometry_confirmed',
            '/ducted/mission/waypoint_mission/enable_commands',
            '/ducted/mission/waypoint_mission/require_automatic_lease')
        self.activation_gate=next((name+' is not enabled' for name in required_gates
                                   if rospy.get_param(name,False) is not True),'')
        self.stop=threading.Event();self.work=threading.Event();self.pending=None
        self.timeout=float(rospy.get_param('~telemetry_timeout',.5))
        self.rpc_timeout=float(rospy.get_param('~service_timeout',1.))
        self.margin=float(rospy.get_param('~corridor_margin',.1))
        self.max_tilt=float(rospy.get_param('~maximum_tilt',.2))
        if any(not math.isfinite(v) or v<=0 for v in (self.timeout,self.rpc_timeout,self.margin,self.max_tilt)):
            raise ValueError('invalid automatic runtime limits')
        geometry=rospy.get_param('~airframe_geometry')
        if not geometry.get('complete') or geometry.get('frame')!='base_link':
            raise ValueError('complete base_link geometry required')
        d=geometry['dimensions_m']
        self.radius=math.hypot(float(d['front_to_back'])/2,float(d['left_to_right'])/2)
        self.half_height=float(d['top_to_bottom'])/2
        if not math.isfinite(self.radius+self.half_height) or min(self.radius,self.half_height)<=0:
            raise ValueError('invalid airframe dimensions')
        sphere=math.hypot(self.radius,self.half_height)
        self.min_target_agl=(float(rospy.get_param('/ducted_navigation/planner/min_agl',.4))+sphere
            +float(rospy.get_param('/ducted_navigation/planner/floor_clearance',.1))
            +float(rospy.get_param('/ducted_navigation/planner/snapshot_motion_margin',.05)))
        self.max_target_agl=(float(rospy.get_param('/ducted_navigation/planner/max_agl',2.5))-sphere
            -float(rospy.get_param('/ducted_navigation/planner/ceiling_clearance',.1))
            -float(rospy.get_param('/ducted_navigation/planner/snapshot_motion_margin',.05)))
        self.streams={};self.messages={};self.readiness={};self.last_ros=0.
        self.odom_history=deque(maxlen=100)
        self.tf=tf2_ros.Buffer();self.tf_listener=tf2_ros.TransformListener(self.tf)
        topics={
            'fcu':('/mavros/state',State),'landed':('/mavros/extended_state',ExtendedState),
            'odom':('/mavros/odometry/out',Odometry),'rc':('/ducted/rc/state',RCState),
            'control':('/ducted/control/status',FlightControlStatus),'mission':('/ducted/mission/status',MissionStatus),
            'navigation':('/ducted/navigation/status',NavigationStatus),'terrain':('/ducted/terrain/height',TerrainHeight),
            'cloud':(rospy.get_param('~cloud_topic','/ducted/relocalization/registered_scan'),PointCloud2)}
        for name,(topic,kind) in topics.items():
            self.streams[name]=StampedInput(name,self.timeout,self.timeout,.05)
            rospy.Subscriber(topic,kind,lambda m,n=name:self.receive(n,m),queue_size=1)
        for topic in ('/ducted/system/ready','/ducted/external_odometry/ready','/ducted/terrain/ready'):
            rospy.Subscriber(topic,Bool,lambda m,t=topic:self.ready(t,m),queue_size=1)
        self.helper=PersistentDeadlineProcess(bootstrap,(3.,))
        self.mission_lease=rospy.Publisher('/ducted/automatic/mission_lease',Bool,queue_size=1)
        self.flight_lease=rospy.Publisher('/ducted/automatic/flight_lease',Bool,queue_size=1)
        self.pub=rospy.Publisher('/ducted/automatic/status',String,queue_size=1,latch=True)
        rospy.Service('/ducted/automatic/start',Trigger,self.start)
        rospy.Service('/ducted/automatic/cancel',Trigger,self.cancel)
        rospy.on_shutdown(self.shutdown)
        threading.Thread(target=self.worker,daemon=True).start()
        threading.Thread(target=self.loop,daemon=True).start()

    def ready(self,name,message):
        with self.lock:self.readiness[name]=(bool(message.data),monotonic())

    def receive(self,name,message):
        now=rospy.get_time();wall=monotonic();valid=True
        try:
            stamp=message.header.stamp.to_sec()
            if name=='cloud':
                if not message.header.frame_id or message.width*message.height>200000:raise ValueError('invalid cloud size/frame')
                pts=tuple(point_cloud2.read_points(message,field_names=('x','y','z'),skip_nans=False))
                if not pts or any(not all(math.isfinite(v) for v in p) for p in pts):raise ValueError('invalid cloud')
                if message.header.frame_id!='odom':
                    tr=self.tf.lookup_transform('odom',message.header.frame_id,message.header.stamp,rospy.Duration(0))
                    t=tr.transform.translation;q=tr.transform.rotation
                    pts=transform_points(pts,(t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
                message=(stamp,pts)
            elif name=='odom':
                p=message.pose.pose.position;q=message.pose.pose.orientation;v=message.twist.twist.linear
                valid=(message.header.frame_id=='odom' and message.child_frame_id=='base_link'
                       and all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z))
                       and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<.001)
            elif name=='terrain':
                valid=message.valid and message.header.frame_id=='odom' and all(math.isfinite(x) for x in (message.agl,message.ground_z,message.variance)) and message.variance>=0
            elif name=='rc':valid=message.valid and message.mode in ('manual','hold','command')
        except Exception:
            stamp=now;valid=False
        with self.lock:
            if self.streams[name].accept(stamp,wall,valid,now):
                self.messages[name]=message
                if name=='odom':
                    p=message.pose.pose.position
                    self.odom_history.append((stamp,(p.x,p.y,p.z)))
            else:
                self.messages.pop(name,None)
                if name=='odom':self.odom_history.clear()

    def snapshot(self):
        wall=monotonic();now=rospy.get_time();reason=''
        if self.activation_gate:return Observation(reason=self.activation_gate)
        if now<self.last_ros:
            self.messages.clear();self.readiness.clear()
            self.odom_history.clear()
        self.last_ros=now
        for name,stream in self.streams.items():
            if name not in self.messages or not stream.fresh(wall,now):reason=name+' unavailable or stale';break
        for name in ('/ducted/system/ready','/ducted/external_odometry/ready','/ducted/terrain/ready'):
            value,arrival=self.readiness.get(name,(False,-math.inf))
            limit=1.5 if name=='/ducted/system/ready' else .5
            if not value or not 0<=wall-arrival<=limit:reason=name+' unavailable or stale'
        if reason:return Observation(reason=reason)
        m=self.messages;f=m['fcu'];rc=m['rc'];odom=m['odom'];p=odom.pose.pose.position
        q=odom.pose.pose.orientation;v=odom.twist.twist.linear
        if not f.connected or f.system_status not in (3,4):reason='FCU not operational'
        elif rc.kill_switch or rc.mode!='command':reason='RC does not grant automatic authority'
        # Horizontal corridor includes the rotated upper/body projection. A
        # ground return below the present body bottom is not a climb obstacle.
        tilt=math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y))))
        target=self.core.target_z if self.core.state in self.core.ACTIVE else p.z+self.core.config.takeoff_rise
        bottom=p.z-self.half_height*math.cos(tilt)-self.radius*math.sin(tilt)
        top=target+self.half_height+self.margin+self.radius*math.sin(tilt)
        radius=self.radius+self.margin+self.half_height*math.sin(tilt)
        cloud_stamp,points=m['cloud']
        sampled=matched_cloud_pose(cloud_stamp,self.odom_history)
        motion=math.inf if sampled is None else math.dist(sampled,(p.x,p.y,p.z))
        if sampled is not None:
            bottom=min(bottom,sampled[2]-self.half_height-self.radius*math.sin(tilt))
            radius+=math.hypot(sampled[0]-p.x,sampled[1]-p.y)
        corridor=(tilt<=self.max_tilt and motion<=.2
                  and not any(bottom+.02<z<top and (x-p.x)**2+(y-p.y)**2<radius**2 for x,y,z in points))
        if not corridor and self.core.state in ('WAIT_OFFBOARD','WAIT_TAKEOFF','TAKEOFF_REQUEST'):
            rospy.logwarn_throttle(1.,'Takeoff corridor unavailable: tilt=%.3f matched_motion=%.3f cloud_age=%.3f',tilt,motion,now-cloud_stamp)
        if self.core.state in ('WAIT_TAKEOFF','TAKEOFF_REQUEST') and tilt>self.max_tilt:reason='excessive takeoff tilt'
        mission=m['mission'];control=m['control']
        return Observation(healthy=not reason,reason=reason,armed=f.armed,
            on_ground=m['landed'].landed_state==1,in_air=m['landed'].landed_state==2,offboard=f.mode=='OFFBOARD',
            controller=control.state,controller_ready=control.ready,mission=mission.state,
            mission_session=mission.session_id+':'+str(mission.epoch),x=p.x,y=p.y,z=p.z,
            speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),corridor_clear=corridor,
            target_agl_safe=self.min_target_agl<=m['terrain'].agl+self.core.config.takeoff_rise<=self.max_target_agl)

    def enqueue(self,actions):
        for action in actions:self.pending=action;self.work.set()

    def start(self,_request):
        with self.lock:
            accepted,action=self.core.start(self.snapshot(),monotonic())
            if action:self.enqueue((action,))
            return TriggerResponse(accepted,self.core.reason)

    def cancel(self,_request):
        with self.lock:
            self.enqueue(self.core.cancel(monotonic()))
            return TriggerResponse(True,self.core.reason)

    def worker(self):
        while not self.stop.is_set():
            self.work.wait(.1);self.work.clear()
            with self.lock:action=self.pending;self.pending=None
            if action is None:continue
            ready,message=self.helper.ensure_ready(4.)
            with self.lock:
                if not self.core.dispatch_current(action):continue
                if action.command not in ('stop','fault_stop'):
                    self.enqueue(self.core.tick(self.snapshot(),monotonic()))
                    if not self.core.dispatch_current(action):continue
                # Dispatch is linearized under the lock; later cancellation
                # supersedes this generation and queues a stop after this call.
            accepted=False
            if ready:
                accepted,message=self.helper.call((action.command,action.z),self.rpc_timeout)
            with self.lock:self.core.ack(action.generation,action.command,accepted,message,monotonic())

    def loop(self):
        while not self.stop.wait(.05):
            with self.lock:
                observation=self.snapshot()
                self.enqueue(self.core.tick(observation,monotonic()))
                self.mission_lease.publish(Bool(data=observation.healthy and self.core.state in ('MISSION_REQUEST','WAIT_MISSION','RUNNING')))
                self.flight_lease.publish(Bool(data=observation.healthy and self.core.state in self.core.ACTIVE+('SUCCEEDED','STOPPING','ABORTED','FAULT')))
                status=dict(state=self.core.state,reason=self.core.reason,generation=self.core.generation,
                            target_z=self.core.target_z,finish=self.core.config.finish)
            self.pub.publish(String(data=json.dumps(status)))

    def shutdown(self):
        self.stop.set();self.work.set();self.helper.close()


if __name__=='__main__':
    rospy.init_node('automatic_flight')
    AutomaticNode();rospy.spin()
