import unittest
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_runtime import runtime, populate, Pose


class PlannerContextTests(unittest.TestCase):
    def ready(self, reference=.4, request='session-1'):
        s = runtime(height_mode='takeoff_relative', require_planner_context=True)
        populate(s)
        self.assertTrue(s.accept_context(10.01,20.01,request,True,True,reference,10.01))
        self.assertTrue(s.set_goal(request,Pose(1,0,1.4,0),10.02,10.02).accepted)
        return s

    def test_context_required_and_wrong_goal_rejected(self):
        s = runtime(height_mode='takeoff_relative', require_planner_context=True)
        self.assertFalse(s.set_goal('r',Pose(1,0,1,0),10,10).accepted)
        s=self.ready()
        self.assertFalse(s.set_goal('old',Pose(9,0,1,0),10.03,10.03).accepted)
        self.assertEqual(s.request_id,'session-1')

    def test_dynamic_reference_is_not_measured_ground(self):
        s=self.ready()
        token,snapshot,reason=s.snapshot(10.03,20.03)
        self.assertEqual(reason,'')
        self.assertAlmostEqual(snapshot.terrain.ground_z,.4)
        self.assertEqual(snapshot.terrain.source,'TAKEOFF_DATUM')
        self.assertTrue(s.revalidate(token,10.03,20.03))

    def test_revocation_invalidates_pending_work_and_next_round_changes_reference(self):
        s=self.ready();token,_,_=s.snapshot(10.03,20.03)
        s.accept_context(10.04,20.04,'',False,False,0,10.04)
        self.assertFalse(s.revalidate(token,10.04,20.04))
        self.assertEqual(s.request_id,'')
        s.accept_context(10.05,20.05,'session-2',True,True,2,10.05)
        self.assertTrue(s.set_goal('session-2',Pose(1,0,3,0),10.06,10.06).accepted)
        _,snapshot,_=s.snapshot(10.07,20.07)
        self.assertAlmostEqual(snapshot.terrain.ground_z,2)

    def test_changed_context_during_transform_rejects_completion(self):
        s=self.ready()
        token,result=s.begin_goal('session-1',10.03,10.03)
        self.assertTrue(result.accepted)
        s.accept_context(10.04,20.04,'session-2',True,True,.4,10.04)
        self.assertFalse(s.complete_goal(token,Pose(1,0,1.4,0),(0,0,0,1),10.05).accepted)

    def test_context_duplicate_and_missing_reference_revoke(self):
        s=self.ready();token,_,_=s.snapshot(10.03,20.03)
        self.assertFalse(s.accept_context(10.01,20.04,'session-1',True,True,.4,10.04))
        self.assertFalse(s.revalidate(token,10.04,20.04))
        self.assertFalse(s.accept_context(10.05,20.05,'session-1',True,False,0,10.05))

    def test_heartbeat_does_not_invalidate_active_plan(self):
        s=self.ready();token,_,_=s.snapshot(10.03,20.03)
        s.accept_context(10.04,20.04,'session-1',True,True,.4,10.04)
        self.assertTrue(s.revalidate(token,10.04,20.04))


if __name__=='__main__':unittest.main()
