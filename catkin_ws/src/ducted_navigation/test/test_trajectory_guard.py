import math
import threading
import unittest
from dataclasses import replace
from ducted_navigation.planner import PlannerConfig,Hull,Pose,Vec3,Terrain,Snapshot
from ducted_navigation.trajectory_guard import TrajectoryGuard
from ducted_navigation.bounded_rpc import BoundedRPC


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.guard=TrajectoryGuard(PlannerConfig(geometry_confirmed=True),Hull(
            ((-.3,-.35,-.1),(.3,-.35,-.1),(-.3,.35,.1),(.3,.35,.1)),
            "base_link",declared_test_geometry=True))
        self.snapshot=Snapshot(1.,Pose(0,0,1.2,0),Vec3(0,0,0),Pose(2,0,1.2,0),
                               ((0,0,0),(4,0,1.2)),Terrain(0,1.2,.001,True))

    def test_reference_is_not_terminal_goal(self):
        result=self.guard.plan_reference(self.snapshot,Pose(.2,0,1.2,0),2)
        self.assertTrue(result.publish_target)
        self.assertEqual(result.state,"AVOIDING")
        self.assertAlmostEqual(result.target.z,1.2)

    def test_absolute_height_is_not_ground_following(self):
        result=self.guard.plan_reference(self.snapshot,Pose(.2,0,1.25,0),2)
        self.assertTrue(result.publish_target)
        self.assertGreater(result.target.z,1.2)
        self.assertAlmostEqual(result.candidate_agl,1.2+result.target.z-1.2)

    def test_live_person_is_not_removed_by_static_map(self):
        s=replace(self.snapshot,obstacles=((.25,0,1.2),))
        self.assertFalse(self.guard.plan_reference(s,Pose(.2,0,1.2,0),2).publish_target)

    def test_invalid_geometry_agl_and_reference(self):
        for reference in (Pose(.2,0,0,0),Pose(math.nan,0,1.2,0)):
            self.assertFalse(self.guard.plan_reference(self.snapshot,reference,2).publish_target)
        self.guard.config=replace(self.guard.config,geometry_confirmed=False)
        self.assertFalse(self.guard.plan_reference(self.snapshot,Pose(.2,0,1.2,0),2).publish_target)

    def test_no_progress_stops_output(self):
        result=None
        for i in range(30):
            result=self.guard.plan_reference(replace(self.snapshot,stamp=1+i*.1),Pose(.2,0,1.2,0),2)
        self.assertFalse(result.publish_target)
        self.assertIn("progress timeout",result.reason)

    def test_goal_approach_brakes_without_dropping_stream(self):
        x=0.;velocity=0.
        for i in range(180):
            snapshot=replace(self.snapshot,stamp=1.+i*.1,current=Pose(x,0,1.2,0),
                             velocity=Vec3(velocity,0,0),goal=Pose(.7,0,1.2,0))
            reference=Pose(min(.7,x+.3),0,1.2,0)
            result=self.guard.plan_reference(snapshot,reference,abs(.7-x))
            self.assertTrue(result.publish_target,(i,x,result.reason))
            for _ in range(5):
                desired=max(-.4,min(.4,6*(result.target.x-x)))
                velocity+=max(-.01,min(.01,desired-velocity))
                x+=velocity*.02
        self.assertLess(abs(x-.7),.02)
        self.assertLess(abs(velocity),.03)

    def test_rpc_timeout_does_not_spawn_more_workers_or_reuse_old_result(self):
        release=threading.Event();calls=[]
        def call(value): calls.append(value);release.wait(1);return value
        rpc=BoundedRPC(call,.01)
        with self.assertRaises(RuntimeError):rpc.request("old")
        with self.assertRaises(RuntimeError):rpc.request("new")
        self.assertEqual(calls,["old"])
        release.set();rpc._pending[0].wait(1)
        self.assertEqual(rpc.request("fresh"),"fresh")


if __name__=="__main__":unittest.main()
