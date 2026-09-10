import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock, patch


PKG = Path(__file__).resolve().parents[1]


def load_script(name, modules):
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location(name, PKG / 'scripts' / (name + '.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class StartupCheckTest(unittest.TestCase):
    NOW = 1800000000.

    def setUp(self):
        self.ros = MagicMock()
        self.ros.myargv.return_value = ['startup_check.py', '--duration', '1']
        self.ros.get_time.return_value = self.NOW
        self.ros.get_name.return_value = '/ducted_startup_check_1'
        self.ros.get_param.side_effect = lambda name, default=None: default
        self.ros.AnyMsg = object
        self.ros.Time.return_value = 0
        self.ros.Duration.side_effect = lambda seconds: seconds
        self.buffer = MagicMock()
        self.buffer.can_transform.return_value = True
        self.tf2 = NS(Buffer=MagicMock(return_value=self.buffer),
                      TransformListener=MagicMock())
        self.rosgraph = NS(Master=MagicMock())
        self.module = load_script(
            'startup_check',
            {'rospy': self.ros, 'rosgraph': self.rosgraph, 'tf2_ros': self.tf2,
             'mavros_msgs.msg': NS(RCIn=object, State=object),
             'nav_msgs.msg': NS(Odometry=object),
             'std_msgs.msg': NS(Bool=object),
             'ducted_msgs.msg': NS(TerrainHeight=object)})
        for obj, name, value in [(self.module.time, 'sleep', MagicMock()),
                                 (self.module.time, 'monotonic', MagicMock(return_value=10.)),
                                 (self.module.os, 'access', MagicMock(return_value=True))]:
            patcher = patch.object(obj, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.ros.Subscriber.side_effect = self._subscribe

    def _subscribe(self, topic, _kind, callback, **_kwargs):
        stamp = NS(to_sec=lambda: self.NOW)
        messages = {
            '/mavros/state': NS(connected=True),
            '/mavros/rc/in': NS(header=NS(stamp=stamp), channels=[1500] * 12, rssi=255),
            '/livox/lidar': NS(),
            '/ducted/system/ready': NS(data=True),
        }
        callback(messages[topic])
        return MagicMock()

    @staticmethod
    def _proxy(system_state=None, system_error=None):
        proxy = MagicMock()
        proxy.__enter__.return_value = proxy
        proxy.__exit__.return_value = False
        proxy.getPid.return_value = (1, '', 123)
        if system_error:
            proxy.getSystemState.side_effect = system_error
        else:
            proxy.getSystemState.return_value = (
                1, '', [[('/ducted/localization/odom', ['/adapter'])], [], []])
        return proxy

    def test_uses_bounded_xmlrpc_and_nonblocking_tf_queries(self):
        proxy = self._proxy()
        self.module.ServerProxy = MagicMock(return_value=proxy)
        self.rosgraph.Master.side_effect = AssertionError('unbounded rosgraph master used')

        with contextlib.redirect_stdout(io.StringIO()):
            result = self.module.main()

        self.assertEqual(result, 0)
        self.assertEqual(self.module.ServerProxy.call_count, 2)
        self.rosgraph.Master.assert_not_called()
        self.assertEqual(self.buffer.can_transform.call_count, 2)
        self.assertTrue(all(call.args[3] == 0 for call in self.buffer.can_transform.call_args_list))

    def test_midrun_master_failure_is_structured_json(self):
        proxy = self._proxy(system_error=OSError('master disappeared'))
        self.module.ServerProxy = MagicMock(return_value=proxy)
        self.rosgraph.Master.return_value.getSystemState.side_effect = OSError('master disappeared')
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / 'startup.json'
            self.ros.myargv.return_value += ['--output', str(output)]
            stdout = io.StringIO()
            try:
                with contextlib.redirect_stdout(stdout):
                    result = self.module.main()
            except Exception as error:
                self.fail('startup failure escaped instead of producing JSON: %r' % error)

            report = json.loads(stdout.getvalue())
            self.assertEqual(result, 1)
            self.assertFalse(report['passed'])
            self.assertIn('master disappeared', report['error'])
            self.assertEqual(json.loads(output.read_text()), report)

    def test_unwritable_report_still_emits_json_failure(self):
        output = MagicMock()
        output.write_text.side_effect = PermissionError('read only')
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            result = self.module.emit_report({'passed': True}, output)
        self.assertEqual(result, 1)
        report = json.loads(stdout.getvalue())
        self.assertFalse(report['passed'])
        self.assertIn('read only', report['output_error'])


class StubbornProcess:
    def __init__(self, bag_path):
        self.returncode = None
        self.wait_timeouts = []
        self.signals = []
        self.terminated = False
        self.killed = False
        bag_path.write_bytes(b'bag evidence')

    def poll(self):
        return self.returncode

    def send_signal(self, value):
        self.signals.append(value)

    def terminate(self):
        self.terminated = True

    def kill(self):
        self.killed = True

    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        if timeout in (10, 3):
            raise subprocess.TimeoutExpired('rosbag', timeout)
        self.returncode = -9
        return self.returncode


class RecordTopicsTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.ros = MagicMock()
        params = {'~directory': str(self.root), '~base_timeout': 2., '~split_mb': 1,
                  '~topics': ['/ducted/system/ready'],
                  '~base_ready_topic': '/ducted/system/ready'}
        self.ros.get_param.side_effect = lambda name, default=None: params.get(name, default)
        self.ros.Subscriber.side_effect = self._subscribe
        self.ros.is_shutdown.side_effect = [False, False, True]
        self.module = load_script(
            'record_topics',
            {'rospy': self.ros, 'std_msgs.msg': NS(Bool=object),
             'roslib.packages': NS(find_node=lambda *_: ['/fake/rosbag/record'])})
        patcher = patch.object(self.module.time, 'monotonic', return_value=10.)
        patcher.start()
        self.addCleanup(patcher.stop)

    def set_popen(self, **kwargs):
        patcher = patch.object(self.module.subprocess, 'Popen', **kwargs)
        result = patcher.start()
        self.addCleanup(patcher.stop)
        return result

    @staticmethod
    def _subscribe(_topic, _kind, callback, **_kwargs):
        callback(NS(data=True))
        callback(NS(data=True))
        return MagicMock()

    def test_stubborn_child_is_killed_and_bag_is_preserved(self):
        holder = {}

        def popen(command, **_kwargs):
            self.assertEqual(command[0], '/fake/rosbag/record')
            prefix = Path(command[command.index('-O') + 1])
            process = StubbornProcess(prefix.parent / 'telemetry_0.bag')
            holder['process'] = process
            return process

        self.set_popen(side_effect=popen)
        try:
            result = self.module.main()
        except Exception as error:
            self.fail('cleanup failure escaped from recorder: %r' % error)

        process = holder['process']
        session = next(self.root.iterdir())
        metadata = json.loads((session / 'session.json').read_text())
        self.assertNotEqual(result, 0)
        self.assertTrue(process.terminated)
        self.assertTrue(process.killed)
        self.assertEqual(process.wait_timeouts, [10, 3, None])
        self.assertEqual((session / 'telemetry_0.bag').read_bytes(), b'bag evidence')
        self.assertEqual(metadata['recorder_exit'], -9)
        self.assertEqual(metadata['cleanup'], 'killed after SIGINT and SIGTERM timeouts')

    def test_launch_failure_is_recorded_in_session_metadata(self):
        self.set_popen(side_effect=FileNotFoundError('rosbag missing'))
        try:
            result = self.module.main()
        except Exception as error:
            self.fail('launch failure escaped from recorder: %r' % error)

        session = next(self.root.iterdir())
        metadata = json.loads((session / 'session.json').read_text())
        self.assertNotEqual(result, 0)
        self.assertIsNone(metadata['recorder_exit'])
        self.assertIn('rosbag missing', metadata['error'])
        self.assertIn('stopped_utc', metadata)


if __name__ == '__main__':
    unittest.main()
