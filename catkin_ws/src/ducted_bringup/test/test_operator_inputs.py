from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from ducted_bringup.operator_inputs import wait_for_fresh_message


def message(stamp,armed=False):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:stamp)),armed=armed)


class FakeRos:
    def __init__(self,initial,later=None):
        self.initial,self.later=initial,later
        self.now=100.;self.subscriptions=0;self.closed=False
    def get_time(self):return self.now
    def is_shutdown(self):return False
    def Subscriber(self,topic,kind,callback,queue_size):
        self.subscriptions+=1
        callback(self.initial)
        if self.later:
            def live():
                self.now=100.02
                callback(self.later)
            timer=threading.Timer(.01,live);timer.daemon=True;timer.start()
        return SimpleNamespace(unregister=lambda:setattr(self,'closed',True))


class OperatorInputsTests(unittest.TestCase):
    def test_fcu_cached_unarmed_state_waits_for_new_live_armed_heartbeat(self):
        ros=FakeRos(message(99.2),message(100.01,True))
        result=wait_for_fresh_message(ros,'/mavros/state',object,wait_timeout=.2)
        self.assertTrue(result.armed)
        self.assertEqual(ros.subscriptions,1)
        self.assertTrue(ros.closed)

    def test_cached_recent_fcu_state_also_does_not_decide_arm_state(self):
        ros=FakeRos(message(99.99),message(100.01,True))
        self.assertTrue(wait_for_fresh_message(ros,'/mavros/state',object,wait_timeout=.2).armed)

    def test_stale_pose_can_recover_without_repeated_subscriptions(self):
        ros=FakeRos(message(99.),message(100.01))
        result=wait_for_fresh_message(ros,'/pose',object,wait_timeout=.2)
        self.assertEqual(result.header.stamp.to_sec(),100.01)
        self.assertEqual(ros.subscriptions,1)

    def test_stale_future_and_nonfinite_streams_expire_and_unsubscribe(self):
        for stamp in (99.,101.,float('nan'),0.):
            ros=FakeRos(message(stamp))
            with self.subTest(stamp=stamp),self.assertRaises(RuntimeError):
                wait_for_fresh_message(ros,'/pose',object,wait_timeout=.02)
            self.assertTrue(ros.closed)


if __name__=='__main__':unittest.main()
