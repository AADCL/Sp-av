#!/usr/bin/env python3
"""Production registration + TF owner on isolated master 11328; no real FCU access."""
import copy
import json
import os
import pathlib
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client

PORT = 11328
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % PORT
os.environ['ROS_IP'] = '127.0.0.1'
os.environ['ROS_HOSTNAME'] = 'localhost'
os.environ['OMP_NUM_THREADS'] = '2'
import numpy as np
import yaml
from registration_fixture import fixture


def main():
    workspace = pathlib.Path(__file__).resolve().parents[3]
    out = workspace/'logs/global_relocalization_isolated'
    out.mkdir(parents=True, exist_ok=True)
    sock = socket.socket(); sock.settimeout(.2)
    if sock.connect_ex(('127.0.0.1', PORT)) == 0: raise RuntimeError('isolated port already in use')
    sock.close()
    children, handles, checks = [], [], []
    stop = threading.Event()
    result = {'checks': checks, 'real_fcu_used': False}
    def launch(args, name):
        f = open(str(out/(name+'.log')), 'w'); handles.append(f)
        p = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT, env=os.environ.copy(), start_new_session=True)
        children.append(p); return p
    def wait(predicate, message, timeout=40):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if predicate():
                checks.append(message); print('PASS', message, flush=True); return
            time.sleep(.05)
        raise AssertionError(message)
    try:
        launch(['roscore', '-p', str(PORT)], 'master')
        def master_ready():
            try: return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0] == 1
            except OSError: return False
        wait(master_ready, 'isolated ROS master started', 10)
        import rospy
        from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
        from mavros_msgs.msg import State
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs.point_cloud2 import create_cloud_xyz32
        from std_msgs.msg import Bool, String, Header
        from std_srvs.srv import Trigger
        import tf2_ros
        from tf.transformations import quaternion_from_euler, quaternion_from_matrix
        sys.path.insert(0, str(workspace/'src/ducted_localization/src'))
        from ducted_localization.core import pose_matrix
        rospy.init_node('global_relocalization_isolated_test', disable_signals=True)
        map_points, local_points, expected = fixture()
        map_file = out/'synthetic_map.pcd'
        with map_file.open('w') as f:
            f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n' % (len(map_points), len(map_points)))
            np.savetxt(f, map_points, fmt='%.6f')
        mount = yaml.safe_load((workspace/'src/ducted_bringup/config/external_odometry.yaml').read_text())
        cfg = yaml.safe_load((workspace/'src/ducted_localization/config/global_relocalization.yaml').read_text())
        cfg.update({'map_file': str(map_file), 'submap_duration': .4, 'timeout': 40., 'job_timeout': 15.,
                    'extrinsic': mount['extrinsic'], 'auto_initialize': True})
        rospy.set_param('/global_relocalizer', cfg)
        rospy.set_param('/world_tf_owner', mount)
        status = {}; ready = {'value': False}; sample = {}
        rospy.Subscriber('/ducted/relocalization/status', String, lambda m:status.update(json.loads(m.data)))
        rospy.Subscriber('/ducted/localization/map_ready', Bool, lambda m:ready.update(value=m.data))
        rospy.Subscriber('/ducted/relocalization/global_odom', Odometry, lambda m:sample.update(odom=m))
        pub = {
            'state': rospy.Publisher('/mavros/state', State, queue_size=1),
            'base': rospy.Publisher('/ducted/system/ready', Bool, queue_size=1),
            'odom': rospy.Publisher('/ducted/localization/odom', Odometry, queue_size=10),
            'body': rospy.Publisher('/ducted/localization/body_odom', Odometry, queue_size=10),
            'cloud': rospy.Publisher('/ducted/localization/cloud_registered', PointCloud2, queue_size=1),
            'manual': rospy.Publisher('/ducted/relocalization/initialpose', PoseWithCovarianceStamped, queue_size=1)}
        flags = {'cloud': True, 'armed': False}
        extrinsic = mount['extrinsic']['base_to_livox']
        base_body = pose_matrix(extrinsic['translation'], quaternion_from_euler(*extrinsic['rpy']))
        local_body = pose_matrix([.7, -.3, .1], quaternion_from_euler(.03, .4567, .15))
        local_base = local_body @ np.linalg.inv(base_body)
        map_base = expected @ local_base
        cloud = create_cloud_xyz32(Header(frame_id='camera_init'), local_points)
        def set_pose(message, matrix):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            p.x, p.y, p.z = matrix[:3, 3]
            q.x, q.y, q.z, q.w = quaternion_from_matrix(matrix)
        static = tf2_ros.StaticTransformBroadcaster()
        dynamic = tf2_ros.TransformBroadcaster()
        identity = TransformStamped(); identity.header.frame_id='odom'; identity.child_frame_id='camera_init'; identity.transform.rotation.w=1.
        body_tf = TransformStamped(); body_tf.header.frame_id='body'; body_tf.child_frame_id='base_link'
        m = np.linalg.inv(base_body); t,q=body_tf.transform.translation,body_tf.transform.rotation
        t.x,t.y,t.z=m[:3,3]; q.x,q.y,q.z,q.w=quaternion_from_matrix(m)
        static.sendTransform([identity, body_tf])
        def publisher():
            count = 0
            while not stop.wait(.02):
                stamp = rospy.Time.now()
                odom = Odometry(); odom.header.stamp=stamp; odom.header.frame_id='camera_init'; odom.child_frame_id='body'
                set_pose(odom, local_body); pub['odom'].publish(odom)
                body = copy.deepcopy(odom); body.header.frame_id='odom'; body.child_frame_id='base_link'; set_pose(body,local_base); pub['body'].publish(body)
                transform=TransformStamped(); transform.header=odom.header; transform.child_frame_id='body'
                transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=local_body[:3,3]
                transform.transform.rotation=odom.pose.pose.orientation; dynamic.sendTransform(transform)
                if count % 5 == 0:
                    state = State(); state.header.stamp=stamp; state.connected=True; state.armed=flags['armed']; state.mode='STABILIZED'
                    pub['state'].publish(state)
                    if flags['cloud']: cloud.header.stamp=stamp; pub['cloud'].publish(cloud)
                if count % 20 == 0: pub['base'].publish(True)
                count += 1
        thread = threading.Thread(target=publisher, daemon=True); thread.start()
        launch([sys.executable, str(workspace/'src/ducted_bringup/scripts/world_tf_owner.py'),
            'local:=/ducted/localization/body_odom', 'global:=/ducted/relocalization/global_odom',
            'localized:=/ducted/relocalization/ready', 'base_ready:=/ducted/system/ready',
            'ready:=/ducted/localization/map_ready'], 'tf_owner')
        launch([str(workspace/'.venv-localization/bin/python3'), str(workspace/'src/ducted_localization/scripts/global_relocalizer.py')], 'global')
        wait(lambda:status.get('success') and status.get('source')=='AUTO' and ready['value'], 'unknown-pose search succeeds and map TF is ready')
        wait(lambda:'odom' in sample, 'global odometry published')
        error = np.linalg.norm(np.asarray(status['position'])-map_base[:3,3])
        if error > .06: raise AssertionError('map/base position error '+str(error))
        checks.append('global pose compensates tilted radar mount'); result['position_error_m']=float(error)
        buffer=tf2_ros.Buffer(); listener=tf2_ros.TransformListener(buffer)
        def tf_ok():
            try:
                transform=buffer.lookup_transform('map','base_link',rospy.Time(0),rospy.Duration(0))
                p=transform.transform.translation
                return np.linalg.norm(np.array([p.x,p.y,p.z])-map_base[:3,3]) < .06
            except Exception: return False
        wait(tf_ok, 'map -> odom -> camera_init -> body -> base_link remains connected')
        time.sleep(2.3)
        if not ready['value'] or not status.get('success'): raise AssertionError('tracking lost on unchanged scene')
        checks.append('tracking and TF continue after first success')
        flags['cloud']=False
        wait(lambda:not ready['value'] and status.get('state')=='LOST', 'cloud dropout revokes global localization', 4)
        flags['cloud']=True; time.sleep(1.)
        if status.get('success'): raise AssertionError('lost localization restarted without explicit request')
        checks.append('dropout recovery does not silently restart global search')
        retry=rospy.ServiceProxy('/ducted/relocalization/retry',Trigger)
        flags['armed']=True; time.sleep(.3)
        if retry().success: raise AssertionError('armed retry accepted')
        checks.append('initialization rejected while fake FCU armed')
        flags['armed']=False; time.sleep(.3)
        if not retry().success: raise AssertionError('disarmed retry rejected')
        wait(lambda:status.get('state')=='SEARCHING', 'explicit retry starts a new search', 4)
        manual=PoseWithCovarianceStamped(); manual.header.frame_id='map'; manual.header.stamp=rospy.Time.now()
        set_pose(manual,map_base); pub['manual'].publish(manual)
        wait(lambda:status.get('source')=='MANUAL', 'RViz manual pose supersedes active automatic search', 5)
        wait(lambda:status.get('success') and ready['value'], 'manual pose converges and continuous tracking resumes')
        bad=copy.deepcopy(manual); bad.header.stamp=rospy.Time.now(); bad.header.frame_id='wrong_frame'
        pub['manual'].publish(bad); time.sleep(.3)
        if not ready['value']: raise AssertionError('invalid manual input revoked valid tracking')
        checks.append('wrong-frame RViz input rejected without replacing valid estimate')
        rospy.sleep(.1)
        nodes=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/audit')[2]
        forbidden=[topic for topic, names in nodes[0] if 'setpoint' in topic or 'cmd_vel' in topic]
        if forbidden: raise AssertionError('unexpected motion publishers '+str(forbidden))
        checks.append('no flight or motion setpoint publishers')
        result['passed']=True
    except Exception as error:
        result['passed']=False; result['error']=repr(error)
        raise
    finally:
        stop.set()
        for p in reversed(children):
            if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
        for p in reversed(children):
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL); p.wait(timeout=2)
        for f in handles:f.close()
        result['children_stopped']=all(p.poll() is not None for p in children)
        (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        print(json.dumps(result,ensure_ascii=False),flush=True)
    return 0


if __name__=='__main__':sys.exit(main())
