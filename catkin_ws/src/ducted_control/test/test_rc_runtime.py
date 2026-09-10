import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_rc import config

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'src'))


class RCRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.ros = MagicMock()
        self.ros.get_time.return_value = 100.
        self.ros.Time.now.return_value = NS(to_sec=lambda: 100.)
        self.params = {'~rc': config(), '~mapping_confirmed': True}
        self.ros.get_param.side_effect = lambda k, d=None: self.params.get(k, d)
        self.ros.Publisher.side_effect = lambda *a, **k: MagicMock()
        messages = NS(RCState=lambda: NS(header=NS()))
        with patch.dict(sys.modules, {'rospy': self.ros, 'ducted_msgs.msg': messages,
                                     'mavros_msgs.msg': NS(RCIn=object),
                                     'std_msgs.msg': NS(Bool=lambda data: NS(data=data))}):
            spec = importlib.util.spec_from_file_location('rc_monitor_tested', PKG / 'scripts/rc_monitor.py')
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)
        self.clock = patch.object(self.module, 'monotonic', return_value=10.)
        self.wall = self.clock.start(); self.addCleanup(self.clock.stop)
        self.node = self.module.RCMonitor()
        self.node._base_callback(NS(data=True))
        self.raw = NS(header=NS(stamp=NS(to_sec=lambda: 100.)), rssi=255,
                      channels=[1494]*4+[1044]*8)

    def test_valid_frame_published_without_any_flight_service(self):
        self.node._rc_callback(self.raw)
        self.assertTrue(self.node.state_pub.publish.call_args[0][0].valid)
        self.node.ready_pub.publish.assert_called_with(True)
        self.ros.ServiceProxy.assert_not_called()

    def test_unconfirmed_mapping_never_ready(self):
        self.params['~mapping_confirmed'] = False
        node = self.module.RCMonitor(); node._base_callback(NS(data=True)); node._rc_callback(self.raw)
        self.assertFalse(node.state_pub.publish.call_args[0][0].valid)
        node.ready_pub.publish.assert_called_with(False)

    def test_kill_level_blocks_ready_but_remains_visible(self):
        self.raw.channels[7] = 1944
        self.node._rc_callback(self.raw)
        output = self.node.state_pub.publish.call_args[0][0]
        self.assertTrue(output.valid)
        self.assertTrue(output.kill_switch)
        self.node.ready_pub.publish.assert_called_with(False)

    def test_base_loss_immediately_invalidates(self):
        self.node._rc_callback(self.raw)
        self.node._base_callback(NS(data=False))
        self.assertFalse(self.node.state_pub.publish.call_args[0][0].valid)

    def test_frozen_ros_clock_stale_rc_and_base_clear_ready(self):
        self.node._rc_callback(self.raw)
        self.wall.return_value = 11.1
        self.node._watchdog()
        self.assertFalse(self.node.state_pub.publish.call_args[0][0].valid)
        self.node.ready_pub.publish.assert_called_with(False)


if __name__ == '__main__': unittest.main()
