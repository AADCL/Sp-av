import importlib.util
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_odometry_runtime import Stamp, odometry, transform

PKG = Path(__file__).resolve().parents[1]


class WorldRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.ros, self.tf = MagicMock(), MagicMock()
        self.ros.Time.now.return_value = Stamp(100.)
        self.ros.Publisher.side_effect = lambda *a, **k: MagicMock()
        params = {'~extrinsic/base_to_livox/translation': [0.,0.,0.],
                  '~extrinsic/base_to_livox/rpy': [0.,0.,0.]}
        self.ros.get_param.side_effect = lambda key, default=None: params.get(key,default)
        modules = {'rospy':self.ros,'tf2_ros':self.tf,'geometry_msgs.msg':NS(TransformStamped=transform),
                   'nav_msgs.msg':NS(Odometry=odometry),'std_msgs.msg':NS(Bool=object)}
        with patch.dict(sys.modules,modules):
            spec = importlib.util.spec_from_file_location('world_runtime',PKG/'scripts/world_tf_owner.py')
            self.module = importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        with patch.object(self.module.threading,'Thread'):
            self.node = self.module.WorldTfOwner()
        self.node.base(NS(data=True));self.node.localized(NS(data=True))

    def pair(self, stamp=100., global_x=11.):
        local, global_ = odometry(), odometry()
        local.header.frame_id,local.child_frame_id = 'odom','base_link'
        global_.header.frame_id,global_.child_frame_id = 'map','body'
        local.header.stamp=global_.header.stamp=Stamp(stamp)
        local.pose.pose.position.x=1.;global_.pose.pose.position.x=global_x
        return local,global_

    def test_no_world_tf_until_valid_matching_pair(self):
        local,global_=self.pair()
        self.node.receive(global_,True)
        self.node.tf.sendTransform.assert_not_called()
        self.node.receive(local,False)
        sent=self.node.tf.sendTransform.call_args[0][0]
        self.assertEqual((sent.header.frame_id,sent.child_frame_id),('map','odom'))
        self.assertAlmostEqual(sent.transform.translation.x,10.)

    def test_nonmatching_source_times_never_pair(self):
        local,global_=self.pair();global_.header.stamp=Stamp(99.99)
        self.node.receive(local,False);self.node.receive(global_,True)
        self.node.tf.sendTransform.assert_not_called()

    def test_global_correction_does_not_modify_local_pose(self):
        for stamp,x in [(99.9,11.),(100.,16.)]:
            local,global_=self.pair(stamp,x)
            self.node.receive(local,False);self.node.receive(global_,True)
            self.assertEqual(local.pose.pose.position.x,1.)
        self.assertAlmostEqual(self.node.tf.sendTransform.call_args[0][0].transform.translation.x,15.)

    def test_unlocalized_and_wrong_frames_do_not_publish(self):
        local,global_=self.pair()
        self.node.localized(NS(data=False))
        self.node.receive(local,False);self.node.receive(global_,True)
        self.node.localized(NS(data=True));global_.child_frame_id='base_link'
        self.node.receive(local,False);self.node.receive(global_,True)
        self.node.tf.sendTransform.assert_not_called()

    def test_watchdog_revokes_ready_after_clock_rollback(self):
        local,global_=self.pair();self.node.receive(local,False);self.node.receive(global_,True)
        self.ros.Time.now.return_value=Stamp(99.)
        self.node.stop=MagicMock();self.node.stop.wait.side_effect=[False,True]
        self.node.watchdog()
        self.node.ready.publish.assert_called_with(False)


if __name__=='__main__':unittest.main()
