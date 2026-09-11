#!/usr/bin/env python3
"""Own map->odom only after valid, source-time-paired global/local estimates."""
import math
import threading
from time import monotonic
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ducted_bringup.fastlio_odometry import adapt_odometry_values, quaternion_from_rpy
from ducted_bringup.world_frames import correction


class WorldTfOwner:
    def __init__(self):
        self.lock = threading.RLock()
        self.local, self.global_ = {}, {}
        self.seen = {'local': 0, 'global': 0}
        self.last = 0
        self.last_wall = None
        self.base_wall = None
        self.enabled = False
        self.max_age = float(rospy.get_param('~max_age', .35))
        if not math.isfinite(self.max_age) or self.max_age <= 0:raise ValueError('invalid max_age')
        self.translation = tuple(rospy.get_param('~extrinsic/base_to_livox/translation'))
        self.orientation = quaternion_from_rpy(*rospy.get_param('~extrinsic/base_to_livox/rpy'))
        self.tf = tf2_ros.TransformBroadcaster()
        self.ready = rospy.Publisher('ready', Bool, queue_size=1, latch=True)
        self.ready.publish(False)
        rospy.Subscriber('local', Odometry, lambda m:self.receive(m,False), queue_size=50)
        rospy.Subscriber('global', Odometry, lambda m:self.receive(m,True), queue_size=50)
        rospy.Subscriber('localized', Bool, self.localized, queue_size=1)
        rospy.Subscriber('base_ready', Bool, self.base, queue_size=1)
        self.stop = threading.Event()
        rospy.on_shutdown(self.stop.set)
        threading.Thread(target=self.watchdog, daemon=True).start()

    def invalidate(self):
        self.local.clear(); self.global_.clear()
        self.last_wall = None
        self.ready.publish(False)

    def localized(self, msg):
        with self.lock:
            self.enabled = bool(msg.data)
            if not self.enabled:self.invalidate()

    def base(self, msg):
        with self.lock:
            self.base_wall = monotonic() if msg.data else None
            if not msg.data:self.invalidate()

    def fresh(self, stamp):
        age = (rospy.Time.now()-stamp).to_sec()
        return math.isfinite(age) and -.05 <= age <= self.max_age

    def gates(self):
        return self.enabled and self.base_wall is not None and monotonic()-self.base_wall <= 1.5

    def receive(self, msg, global_input):
        with self.lock:
            name = 'global' if global_input else 'local'
            stamp = msg.header.stamp.to_nsec()
            if not self.gates():self.invalidate();return
            if not self.fresh(msg.header.stamp) or stamp <= self.seen[name]:self.invalidate();return
            self.seen[name] = stamp
            expected = ('map','body') if global_input else ('odom','base_link')
            if (msg.header.frame_id,msg.child_frame_id) != expected:self.invalidate();return
            p,q = msg.pose.pose.position,msg.pose.pose.orientation
            values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
            if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1)> .01:
                self.invalidate();return
            cache = self.global_ if global_input else self.local
            cache[stamp] = msg
            while len(cache)>100:del cache[min(cache)]
            # Both estimators use the same scan-end timestamp. Never pair latest-to-latest.
            if stamp not in self.local or stamp not in self.global_ or stamp <= self.last:return
            local, global_msg = self.local[stamp],self.global_[stamp]
            gp,gq = global_msg.pose.pose.position,global_msg.pose.pose.orientation
            lp,lq = local.pose.pose.position,local.pose.pose.orientation
            try:
                base = adapt_odometry_values((gp.x,gp.y,gp.z),(gq.x,gq.y,gq.z,gq.w),
                    (0,0,0),(0,0,0),self.translation,self.orientation)
                position,orientation = correction(base.position,base.child_orientation,
                    (lp.x,lp.y,lp.z),(lq.x,lq.y,lq.z,lq.w))
            except ValueError:
                self.invalidate();return
            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id,transform.child_frame_id = 'map','odom'
            t,r = transform.transform.translation,transform.transform.rotation
            t.x,t.y,t.z = position
            r.x,r.y,r.z,r.w = orientation
            if not self.gates() or not self.fresh(msg.header.stamp):self.invalidate();return
            self.tf.sendTransform(transform)
            self.last,self.last_wall = stamp,monotonic()
            self.ready.publish(True)
            for cache in (self.local,self.global_):
                for old in list(cache):
                    if old<=stamp:del cache[old]

    def watchdog(self):
        while not self.stop.wait(.05):
            with self.lock:
                age = rospy.Time.now().to_sec()-self.last/1e9
                if not self.gates():self.invalidate()
                elif self.last_wall is None:self.ready.publish(False)
                elif monotonic()-self.last_wall>self.max_age or not -.05<=age<=self.max_age:self.invalidate()


if __name__ == '__main__':
    rospy.init_node('world_tf_owner')
    WorldTfOwner()
    rospy.spin()
