#!/usr/bin/env python3
"""Automatic global registration, RViz manual initialization, continuous map tracking.

This node publishes map/body odometry, never TF or flight commands. The existing
world_tf_owner pairs it with local body odometry at the identical source stamp.
"""
import copy
import json
import math
import multiprocessing as mp
import queue
import threading
from time import monotonic

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, String, Header
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_matrix, quaternion_from_euler
from ducted_localization.core import Session, pose_matrix, initial_correction
from ducted_localization.registration import worker_main


def matrix_pose(pose):
    p, q = pose.position, pose.orientation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def fill_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = matrix[:3, 3]
    q = quaternion_from_matrix(matrix)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q


class GlobalRelocalizer:
    def __init__(self):
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.cfg = dict(rospy.get_param('~'))
        self.map_file = rospy.get_param('~map_file')
        self.session = Session(**{k: self.cfg[k] for k in ('timeout', 'min_fitness', 'max_rmse',
            'required_confirmations', 'tracking_timeout', 'failure_limit', 'max_translation', 'max_rotation')})
        for key in ('input_timeout', 'job_timeout', 'submap_duration', 'tracking_period',
                    'coarse_voxel', 'fine_voxel', 'min_submap_frames', 'min_scan_points', 'max_submap_points'):
            if not math.isfinite(float(self.cfg[key])) or float(self.cfg[key]) <= 0:
                raise ValueError('invalid '+key)
        mount = rospy.get_param('~extrinsic/base_to_livox')
        self.base_body = pose_matrix(mount['translation'], quaternion_from_euler(*mount['rpy']))
        self.odom = self.cloud = self.fcu = None
        self.odom_wall = self.cloud_wall = self.fcu_wall = self.base_wall = None
        self.seen = {'odom': 0, 'cloud': 0}
        self.last_ros = rospy.Time.now().to_sec()
        self.auto_pending = bool(rospy.get_param('~auto_initialize', True))
        self.process = self.requests = self.responses = None
        self.worker_ready = False
        self.worker_started = None
        self.job = None
        self.job_id = 0
        self.next_track = 0.
        self.last_collected = 0
        self.scans = []
        self.collect_started = None
        self.success_reported = False
        self.last_status = 0.
        self.status_pub = rospy.Publisher('/ducted/relocalization/status', String, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher('/ducted/relocalization/ready', Bool, queue_size=1, latch=True)
        self.global_pub = rospy.Publisher('/ducted/relocalization/global_odom', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher('/ducted/relocalization/pose', PoseStamped, queue_size=1)
        self.map_pub = rospy.Publisher('/ducted/relocalization/global_map', PointCloud2, queue_size=1, latch=True)
        self.scan_pub = rospy.Publisher('/ducted/relocalization/registered_scan', PointCloud2, queue_size=1)
        self.ready_pub.publish(False)
        rospy.Subscriber('/ducted/localization/odom', Odometry, self.receive_odom, queue_size=50)
        rospy.Subscriber('/ducted/localization/cloud_registered', PointCloud2, self.receive_cloud, queue_size=2)
        rospy.Subscriber(rospy.get_param('~base_ready_topic', '/ducted/system/ready'), Bool, self.receive_base, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.receive_fcu, queue_size=1)
        rospy.Subscriber('/ducted/relocalization/initialpose', PoseWithCovarianceStamped, self.manual, queue_size=1)
        self.retry = rospy.Service('/ducted/relocalization/retry', Trigger, self.retry_callback)
        rospy.on_shutdown(self.shutdown)
        # Monotonic loop continues enforcing deadlines even if /clock freezes.
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def receive_base(self, msg):
        with self.lock:
            self.base_wall = monotonic() if msg.data else None
            if not msg.data: self.revoke('base system is not ready')

    def receive_fcu(self, msg):
        with self.lock:
            self.fcu, self.fcu_wall = msg, monotonic()
            if not msg.connected: self.revoke('FCU disconnected')
            elif msg.armed and self.session.state == 'SEARCHING': self.revoke('initialization interrupted by arming')

    def valid_stamp(self, stamp):
        return stamp > 0 and -.05 <= rospy.Time.now().to_sec()-stamp <= self.cfg['input_timeout']

    def accept_header(self, msg, name, expected_frame):
        stamp = msg.header.stamp.to_nsec()
        if stamp <= self.seen[name] or not self.valid_stamp(stamp/1e9) or msg.header.frame_id != expected_frame:
            self.revoke(name+' has invalid frame, time, or ordering')
            return False
        self.seen[name] = stamp
        return True

    def receive_odom(self, msg):
        with self.lock:
            if not self.accept_header(msg, 'odom', 'camera_init'):
                self.odom = None
                return
            try:
                if msg.child_frame_id != 'body': raise ValueError('expected body child frame')
                matrix_pose(msg.pose.pose)
            except ValueError as error:
                self.odom = None
                self.revoke(str(error))
                return
            self.odom, self.odom_wall = msg, monotonic()
            if self.healthy() and self.session.fresh(monotonic()):
                global_msg = Odometry()
                global_msg.header = copy.deepcopy(msg.header)
                global_msg.header.frame_id, global_msg.child_frame_id = 'map', 'body'
                global_body = self.session.transform @ matrix_pose(msg.pose.pose)
                fill_pose(global_msg.pose.pose, global_body)
                # This output is pose-only for the TF owner, never fed directly to PX4.
                global_msg.pose.covariance = [0.] * 36
                for index in (0, 7, 14): global_msg.pose.covariance[index] = max(self.session.rmse**2, .0025)
                for index in (21, 28, 35): global_msg.pose.covariance[index] = .01
                self.ready_pub.publish(True)
                self.global_pub.publish(global_msg)
                pose = PoseStamped(header=copy.deepcopy(global_msg.header))
                fill_pose(pose.pose, global_body @ np.linalg.inv(self.base_body))
                self.pose_pub.publish(pose)
                if not self.success_reported:
                    q, p = pose.pose.orientation, pose.pose.position
                    rospy.loginfo('relocalization success=true source=%s frame=map pose=(%.3f, %.3f, %.3f) quaternion=(%.6f, %.6f, %.6f, %.6f) fitness=%.4f rmse=%.4f; continuing map tracking',
                        self.session.source, p.x, p.y, p.z, q.x, q.y, q.z, q.w, self.session.fitness, self.session.rmse)
                    self.success_reported = True

    def receive_cloud(self, msg):
        with self.lock:
            if not self.accept_header(msg, 'cloud', 'camera_init'):
                self.cloud = None
                return
            if msg.width*msg.height < self.cfg['min_scan_points']:
                self.cloud = None
                self.revoke('point cloud contains too few points')
                return
            self.cloud, self.cloud_wall = msg, monotonic()

    def healthy(self):
        now = monotonic()
        return (self.base_wall is not None and 0 <= now-self.base_wall <= 1.5
            and self.fcu is not None and self.fcu.connected and self.fcu_wall is not None and 0 <= now-self.fcu_wall <= 1.5
            and self.odom is not None and self.cloud is not None
            and 0 <= now-self.odom_wall <= self.cfg['input_timeout']
            and 0 <= now-self.cloud_wall <= self.cfg['input_timeout']
            and self.valid_stamp(self.odom.header.stamp.to_sec()) and self.valid_stamp(self.cloud.header.stamp.to_sec()))

    def can_initialize(self):
        return self.healthy() and not self.fcu.armed

    def revoke(self, reason):
        if self.session.state in ('SEARCHING', 'TRACKING'):
            self.session.invalidate(reason)
        self.ready_pub.publish(False)

    def start(self, source, seed=None):
        self.stop_worker()
        self.session.begin(source, monotonic(), seed)
        self.auto_pending = False
        self.scans, self.collect_started = [], None
        self.last_collected = self.seen['cloud']
        self.success_reported = False
        self.ready_pub.publish(False)
        self.start_worker()
        rospy.loginfo('Starting %s relocalization; keeping local FAST-LIO unchanged', source)

    def manual(self, msg):
        with self.lock:
            try:
                if not self.can_initialize(): raise ValueError('manual initialization requires fresh data and a connected, disarmed FCU')
                if msg.header.frame_id != 'map': raise ValueError('RViz Fixed Frame must be map')
                if not 0 <= rospy.Time.now().to_sec()-msg.header.stamp.to_sec() <= 2.:
                    raise ValueError('manual initial pose timestamp is stale')
                seed = initial_correction(matrix_pose(msg.pose.pose), matrix_pose(self.odom.pose.pose), self.base_body)
                self.start('MANUAL', seed)
            except ValueError as error:
                rospy.logwarn('Manual initial pose rejected: %s', error)

    def retry_callback(self, _request):
        with self.lock:
            if not self.can_initialize():
                return TriggerResponse(False, 'retry requires fresh data and a connected, disarmed FCU')
            self.start('AUTO')
            return TriggerResponse(True, 'automatic search accepted; final result is on /ducted/relocalization/status')

    def start_worker(self):
        context = mp.get_context('spawn')
        self.requests, self.responses = context.Queue(1), context.Queue(2)
        self.process = context.Process(target=worker_main, args=(self.map_file, self.cfg, self.requests, self.responses), daemon=True)
        self.process.start()
        self.worker_started, self.worker_ready = monotonic(), False

    def stop_worker(self):
        if self.process is not None:
            if self.process.is_alive(): self.process.terminate()
            self.process.join(.15)
            if self.process.is_alive(): self.process.kill(); self.process.join(.15)
            for channel in (self.requests, self.responses):
                channel.cancel_join_thread()
                channel.close()
        self.process = None
        self.worker_ready, self.job = False, None

    @staticmethod
    def xyz(msg):
        return np.asarray(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float64).reshape((-1, 3))

    def publish_cloud(self, publisher, points, stamp):
        if publisher == self.map_pub or publisher.get_num_connections():
            publisher.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='map'), points))

    def receive_results(self):
        if self.responses is None: return
        try: result = self.responses.get_nowait()
        except queue.Empty: return
        if result['kind'] == 'loaded':
            self.worker_ready = True
            self.publish_cloud(self.map_pub, result['points'], rospy.Time.now())
            return
        if result['kind'] == 'fatal':
            self.session.invalidate(result['error'], 'FAILED')
            rospy.logerr('Relocalization backend failed: %s', result['error'])
            return
        if self.job is None or (result['generation'], result['id']) != (self.job['generation'], self.job['id']): return
        job, self.job = self.job, None
        # Recheck at acceptance, after any expensive conversion/publication.
        if not self.healthy() or monotonic()-job['started'] > self.cfg['job_timeout']:
            self.session.invalidate('registration completed after data loss or its deadline')
            return
        if 'error' in result:
            self.session.reject(result['error'])
        else:
            accepted = self.session.accept(result['generation'], result['id'], result['matrix'],
                result['fitness'], result['rmse'], monotonic())
            if accepted:
                points = job['points'] @ result['matrix'][:3, :3].T + result['matrix'][:3, 3]
                self.publish_cloud(self.scan_pub, points, job['stamp'])
        # Confirmation submaps contain new scans acquired after the prior solve.
        self.scans, self.collect_started = [], None
        self.last_collected = self.seen['cloud']
        self.next_track = monotonic()+self.cfg['tracking_period']

    def submit(self, points, stamp):
        if len(points) > self.cfg['max_submap_points']:
            points = points[::int(math.ceil(len(points)/self.cfg['max_submap_points']))]
        self.job_id += 1
        seed = self.session.transform if self.session.state == 'TRACKING' else self.session.seed
        job = {'generation': self.session.generation, 'id': self.job_id,
               'points': points, 'seed': seed, 'stamp': stamp, 'started': monotonic()}
        self.requests.put_nowait({k: job[k] for k in ('generation', 'id', 'points', 'seed')})
        self.job = job

    def step(self):
        now, ros_now = monotonic(), rospy.Time.now().to_sec()
        if ros_now < self.last_ros-.001:
            self.revoke('ROS clock moved backwards; restart localization')
            self.odom = self.cloud = None
        self.last_ros = ros_now
        self.session.tick(now)
        if self.auto_pending and self.can_initialize(): self.start('AUTO')
        if self.session.state in ('SEARCHING', 'TRACKING') and not self.healthy(): self.revoke('local sensor data or base/FCU heartbeat expired')
        if self.session.state == 'SEARCHING' and self.fcu is not None and self.fcu.armed: self.revoke('initialization requires disarmed FCU')
        if self.process is not None:
            self.receive_results()
            if not self.process.is_alive() and self.session.state in ('SEARCHING', 'TRACKING'):
                self.session.invalidate('registration worker exited', 'FAILED')
            elif not self.worker_ready and monotonic()-self.worker_started > self.cfg['timeout']:
                self.session.invalidate('map loading timed out', 'FAILED')
            elif self.job is not None and monotonic()-self.job['started'] > self.cfg['job_timeout']:
                self.session.invalidate('registration worker timed out', 'FAILED')
        if self.session.state not in ('SEARCHING', 'TRACKING'):
            self.stop_worker()
        elif self.worker_ready and self.job is None and self.healthy():
            stamp = self.cloud.header.stamp.to_nsec()
            if stamp > self.last_collected:
                if self.session.state == 'TRACKING':
                    if monotonic() >= self.next_track:
                        self.submit(self.xyz(self.cloud), self.cloud.header.stamp)
                        self.last_collected = stamp
                else:
                    if self.collect_started is None: self.collect_started = monotonic()
                    points = self.xyz(self.cloud)
                    per_frame = max(500, int(self.cfg['max_submap_points']//max(self.cfg['min_submap_frames'], 30)))
                    self.scans.append(points[::max(1, int(math.ceil(len(points)/per_frame)))])
                    self.scans = self.scans[-60:]
                    self.last_collected = stamp
                    if len(self.scans) >= self.cfg['min_submap_frames'] and monotonic()-self.collect_started >= self.cfg['submap_duration']:
                        self.submit(np.concatenate(self.scans), self.cloud.header.stamp)
        valid = self.healthy() and self.session.fresh(monotonic())
        self.ready_pub.publish(valid)
        if monotonic()-self.last_status >= .5:
            data = {'state': self.session.state, 'success': bool(valid), 'source': self.session.source,
                    'reason': self.session.reason, 'fitness': self.session.fitness, 'rmse': self.session.rmse,
                    'confirmations': self.session.confirmations, 'frame_id': 'map', 'child_frame_id': 'base_link'}
            if valid:
                matrix = self.session.transform @ matrix_pose(self.odom.pose.pose) @ np.linalg.inv(self.base_body)
                data['position'] = matrix[:3, 3].tolist()
                data['quaternion_xyzw'] = quaternion_from_matrix(matrix).tolist()
            self.status_pub.publish(json.dumps(data, allow_nan=False))
            self.last_status = monotonic()

    def run(self):
        while not self.stop.wait(.05) and not rospy.is_shutdown():
            try:
                with self.lock: self.step()
            except Exception as error:
                with self.lock:
                    self.session.invalidate(str(error), 'FAILED')
                    self.ready_pub.publish(False)
                    self.stop_worker()
                rospy.logerr_throttle(2., 'Relocalization stopped: %s', error)

    def shutdown(self):
        self.stop.set()
        with self.lock: self.stop_worker()


if __name__ == '__main__':
    rospy.init_node('global_relocalizer')
    GlobalRelocalizer()
    rospy.spin()
