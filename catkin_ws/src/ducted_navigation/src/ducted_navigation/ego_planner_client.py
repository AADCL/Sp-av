"""ROS conversion for the read-only EGO local planning service."""
import math
from time import monotonic
import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from ducted_planning.srv import PlanLocal, PlanLocalRequest
from .bounded_rpc import BoundedRPC
from .planner import Pose
from .failure_recorder import FailureRecorder


class EgoPlannerClient:
    def __init__(self,node):
        self.node=node
        service=rospy.get_param("~planning_service","/ducted/ego/plan")
        self.rpc=BoundedRPC(rospy.ServiceProxy(service,PlanLocal),.30)
        self.failures=FailureRecorder(
            rospy.get_param('~failure_log_dir', '~/catkin_ws/logs/ego_failures'),
            on_error=lambda error: rospy.logwarn_throttle(5., 'EGO failure log: '+error))
        rospy.on_shutdown(self.failures.close)

    def prepare(self,snapshot,request_id):
        guard=self.node.planner
        req=None
        began=monotonic()
        stages={}
        received_age=rospy.Time.now().to_sec()-snapshot.stamp
        try:
            inflation,minimum,maximum=guard.envelope(snapshot)
            stages['envelope']=monotonic()-began
            req=PlanLocalRequest()
            req.header=Header(stamp=rospy.Time.from_sec(snapshot.stamp),frame_id="odom")
            self.node._fill_pose(req.start,snapshot.current)
            self.node._fill_pose(req.goal,snapshot.goal)
            req.velocity.x,req.velocity.y,req.velocity.z=(snapshot.velocity.x,snapshot.velocity.y,snapshot.velocity.z)
            # The EGO core also sees retained and predicted moving
            # obstacles. The static-map filter never supplies this channel.
            expanded=guard._expanded_obstacles(snapshot)
            stages['expanded']=monotonic()-began
            req.obstacles=point_cloud2.create_cloud_xyz32(req.header,expanded)
            req.sensor_horizon=guard.config.sensor_horizon
            req.clearance,req.minimum_z,req.maximum_z=inflation,minimum,maximum
            return req,snapshot,request_id,received_age,began,stages,guard.config.goal_tolerance
        except Exception as error:
            raise ValueError("EGO planning unavailable: "+str(error)) from error

    def execute(self,packet):
        req,snapshot,request_id,received_age,began,stages,tolerance=packet
        try:
            stages['rpc_start']=monotonic()-began
            response=self.rpc.request(req)
            stages['rpc_end']=monotonic()-began
            if not response.success:
                raise ValueError(response.reason)
            if (response.local_path.header.frame_id!="odom"
                    or response.local_path.header.stamp!=req.header.stamp
                    or len(response.local_path.poses)<2):
                raise ValueError("EGO response frame or source stamp mismatch")
            route_length=math.sqrt((snapshot.goal.x-snapshot.current.x)**2
                                  +(snapshot.goal.y-snapshot.current.y)**2
                                  +(snapshot.goal.z-snapshot.current.z)**2)
            p=response.tracking_target
            yaw=(snapshot.goal.yaw if route_length<tolerance
                 else math.atan2(p.y-snapshot.current.y,p.x-snapshot.current.x))
            return Pose(p.x,p.y,p.z,yaw)
        except Exception as error:
            if req is not None:
                self.failures.submit(req, dict(request_id=request_id, reason=str(error),
                    source_stamp=snapshot.stamp, age_at_start=received_age,
                    age_at_failure=rospy.Time.now().to_sec()-snapshot.stamp,
                    duration=monotonic()-began, raw_points=len(snapshot.obstacles),
                    sent_points=req.obstacles.width * req.obstacles.height, stages=stages))
            raise ValueError("EGO planning unavailable: "+str(error)) from error

    def reference(self,snapshot):
        return self.execute(self.prepare(snapshot,self.node.runtime.request_id))
