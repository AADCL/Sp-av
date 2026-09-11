"""ROS conversion for the read-only global/local planning service."""
import math
import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from ducted_planning.srv import PlanPath, PlanPathRequest
from .bounded_rpc import BoundedRPC
from .planner import Pose


class FastPlannerClient:
    def __init__(self,node):
        self.node=node
        service=rospy.get_param("~planning_service","/ducted/global_local/plan")
        self.rpc=BoundedRPC(rospy.ServiceProxy(service,PlanPath),.30)

    def plan(self,snapshot):
        guard=self.node.planner
        try:
            inflation,minimum,maximum=guard.envelope(snapshot)
            req=PlanPathRequest()
            req.header=Header(stamp=rospy.Time.from_sec(snapshot.stamp),frame_id="odom")
            self.node._fill_pose(req.start,snapshot.current)
            self.node._fill_pose(req.goal,snapshot.goal)
            req.velocity.x,req.velocity.y,req.velocity.z=(snapshot.velocity.x,snapshot.velocity.y,snapshot.velocity.z)
            # The global/local core also sees retained and predicted moving
            # obstacles. The static-map filter never supplies this channel.
            expanded=guard._expanded_obstacles(snapshot)
            req.obstacles=point_cloud2.create_cloud_xyz32(req.header,expanded)
            tf=self.node._lookup("odom","map",req.header.stamp,snapshot.stamp)
            req.map_to_odom=tf.transform
            req.clearance,req.minimum_z,req.maximum_z=inflation,minimum,maximum
            response=self.rpc.request(req)
            if not response.success:
                return guard.reject(snapshot,response.reason)
            if (response.local_path.header.frame_id!="odom"
                    or response.local_path.header.stamp!=req.header.stamp
                    or response.global_path.header.frame_id!="odom"
                    or response.global_path.header.stamp!=req.header.stamp
                    or len(response.global_path.poses)<2):
                return guard.reject(snapshot,"planner response frame or source stamp mismatch")
            points=[p.pose.position for p in response.global_path.poses]
            route_length=sum(math.sqrt((b.x-a.x)**2+(b.y-a.y)**2+(b.z-a.z)**2)
                             for a,b in zip(points,points[1:]))
            p=response.tracking_target
            yaw=(snapshot.goal.yaw if route_length<guard.config.goal_tolerance
                 else math.atan2(p.y-snapshot.current.y,p.x-snapshot.current.x))
            return guard.plan_reference(snapshot,Pose(p.x,p.y,p.z,yaw),route_length)
        except Exception as error:
            return guard.reject(snapshot,"global/local planning unavailable: "+str(error))
