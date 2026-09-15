import pathlib,sys,threading,unittest
from types import SimpleNamespace as NS
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src'))
from ducted_navigation.ego_worker import EgoReferenceWorker

class WorkerTest(unittest.TestCase):
    def worker(self):
        self.clock=[10.,20.];self.started=threading.Event();self.release=threading.Event();self.done=threading.Event()
        def execute(packet):
            self.started.set();self.release.wait(2)
            if getattr(self,'failure',None):raise ValueError(self.failure)
            return 'EGO reference'
        client=NS(prepare=lambda s,r:s,execute=execute)
        self.worker_=EgoReferenceWorker(client,self.done.set,lambda:tuple(self.clock))
        self.addCleanup(self.release.set)
        return self.worker_

    def test_waiting_service_does_not_block_valid_reference_checks(self):
        worker=self.worker();s=NS(stamp=10.)
        self.assertIsNone(worker.poll(s,('a',1)))
        self.assertTrue(self.started.wait(1))
        self.release.set();self.assertTrue(self.done.wait(1))
        self.assertEqual(worker.poll(s,('a',1)),('EGO reference',10.))
        self.release.clear();self.done.clear();self.started.clear()
        self.clock=[10.1,20.1]
        self.assertEqual(worker.poll(NS(stamp=10.1),('a',1)),('EGO reference',10.))
        self.assertTrue(self.started.wait(1))
        self.clock=[10.6,20.6]
        self.assertIsNone(worker.poll(NS(stamp=10.2),('a',1)))

    def test_cancelled_or_superseded_result_cannot_reappear(self):
        worker=self.worker();worker.poll(NS(stamp=10.),('a',1));self.assertTrue(self.started.wait(1))
        worker.reset();self.release.set();self.assertTrue(self.done.wait(1))
        self.release.clear();self.done.clear()
        self.assertIsNone(worker.poll(NS(stamp=10.1),('b',2)))

    def test_collision_error_revokes_cached_reference(self):
        worker=self.worker();worker.poll(NS(stamp=10.),('a',1));self.release.set();self.assertTrue(self.done.wait(1))
        self.failure='EGO planning unavailable: local endpoint is occupied'
        self.done.clear();self.release.clear();self.started.clear();self.clock=[10.1,20.1]
        worker.poll(NS(stamp=10.1),('a',1));self.assertTrue(self.started.wait(1))
        self.release.set();self.assertTrue(self.done.wait(1))
        with self.assertRaisesRegex(ValueError,'occupied'):worker.poll(NS(stamp=10.1),('a',1))
        self.assertFalse(worker.commit_reference(('a',1),10.,lambda:True))

    def test_frozen_ros_time_does_not_extend_reference(self):
        worker=self.worker();worker.poll(NS(stamp=10.),('a',1));self.release.set();self.assertTrue(self.done.wait(1))
        self.clock=[10.,20.51]
        self.assertIsNone(worker.poll(NS(stamp=10.),('a',1)))

    def test_new_success_does_not_discard_still_fresh_checked_reference(self):
        worker=self.worker();worker.poll(NS(stamp=10.),('a',1));self.release.set();self.assertTrue(self.done.wait(1))
        self.done.clear();self.clock=[10.1,20.1]
        worker.poll(NS(stamp=10.1),('a',1));self.assertTrue(self.done.wait(1))
        self.assertTrue(worker.commit_reference(('a',1),10.,lambda:True))
        worker.reset()
        self.assertFalse(worker.commit_reference(('a',1),10.,lambda:True))

    def test_only_actual_age_expiry_is_retryable(self):
        worker=self.worker()
        self.assertTrue(worker.expired_error('EGO planning unavailable: invalid or stale EGO snapshot: age=0.51 speed=0'))
        self.assertFalse(worker.expired_error('EGO planning unavailable: invalid or stale EGO snapshot: age=0.1 speed=99'))
        self.assertFalse(worker.expired_error('EGO planning unavailable: invalid or stale EGO snapshot: age=-0.2 speed=0'))

if __name__=='__main__':unittest.main()
