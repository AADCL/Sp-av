import math
import threading
import unittest
from dataclasses import replace
from ducted_navigation.planner import PlannerConfig,Hull,Pose,Vec3,Terrain,Snapshot
from ducted_navigation.trajectory_guard import TrajectoryGuard
from ducted_navigation.bounded_rpc import BoundedRPC
from unittest.mock import patch


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.guard=TrajectoryGuard(PlannerConfig(geometry_confirmed=True),Hull(
            ((-.3,-.35,-.1),(.3,-.35,-.1),(-.3,.35,.1),(.3,.35,.1)),
            "base_link",declared_test_geometry=True))
        self.snapshot=Snapshot(1.,Pose(0,0,1.2,0),Vec3(0,0,0),Pose(2,0,1.2,0),
                               ((0,0,0),(4,0,1.2)),Terrain(0,1.2,.001,True))

    def test_dense_vertical_broad_phase_preserves_scalar_collision_verdicts(self):
        background=tuple((i*.02, 2., -1.) for i in range(400))
        for end_z in (1.2, .4, 2.):
            start,end=Pose(0,0,1.2,0),Pose(1,0,end_z,0)
            for point_z in (0.,.1499999999999,.15,.95,1.2,1.45,2.25,4.):
                for point_x in (.3, 1., 2.):
                    points=background+((point_x,.1,point_z),)
                    fast=self.guard._segment_clearance(start,end,points,.6)
                    point_fast=self.guard._point_collision(start,points,.6)
                    with patch('ducted_navigation.planner._np',None):
                        scalar=self.guard._segment_clearance(start,end,points,.6)
                        point_scalar=self.guard._point_collision(start,points,.6)
                    self.assertEqual(fast,scalar,(end_z,point_x,point_z))
                    self.assertEqual(point_fast,point_scalar)

    def test_dense_numeric_voxel_keys_preserve_centroids_order_and_large_coordinates(self):
        points=tuple((math.sin(i)*3, math.cos(i/3)*2, (i%19)*.013-.2) for i in range(1500))
        for cloud in (points+points, points+((-1e15,0,0),(1e15,0,0))):
            self.assertEqual(self.guard._voxelize(cloud),self.guard._voxelize_python(cloud))

    def test_reference_is_not_terminal_goal(self):
        result=self.guard.plan_reference(self.snapshot,Pose(.2,0,1.2,0),2)
        self.assertTrue(result.publish_target)
        self.assertEqual(result.state,"AVOIDING")
        self.assertAlmostEqual(result.target.z,1.2)

    def test_takeoff_datum_limits_center_height_without_inventing_floor(self):
        self.guard.config=replace(self.guard.config,min_agl=.4,snapshot_motion_margin=.1)
        s=replace(self.snapshot,current=Pose(0,0,.95,0),goal=Pose(2,0,1,0),
                  terrain=Terrain(0,.95,0,True,'TAKEOFF_DATUM'))
        inflation,low,high=self.guard.envelope(s)
        self.assertAlmostEqual(low,.4)
        self.assertAlmostEqual(high,2.5)
        self.assertTrue(self.guard.plan_reference(s,Pose(.2,0,1,0),2).publish_target)
        blocked=replace(s,obstacles=((.2,0,.95),))
        self.assertFalse(self.guard.plan_reference(blocked,Pose(.2,0,1,0),2).publish_target)

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

    def test_replanned_tangent_turns_with_bounded_acceleration(self):
        self.guard.config=replace(self.guard.config,max_acceleration=.5)
        self.guard._previous_command_velocity=(.3,0,0)
        self.guard._previous_speed=.3
        self.guard._previous_stamp=.9
        result=self.guard.plan_reference(self.snapshot,Pose(0,.3,1.2,0),2)
        self.assertTrue(result.publish_target,result.reason)
        v=(result.target.x/.1,result.target.y/.1,(result.target.z-1.2)/.1)
        self.assertLessEqual(math.sqrt((v[0]-.3)**2+v[1]**2+v[2]**2),.05+1e-9)
        self.assertGreater(result.target.x,0)
        self.assertGreater(result.target.y,0)

    def test_actual_turning_chord_is_checked_after_acceleration_slew(self):
        self.guard._previous_command_velocity=(.3,0,0)
        self.guard._previous_speed=.3
        self.guard._previous_stamp=.9
        # Isolate the geometric decision to prove the final deviating chord is
        # validated, even when the proposed EGO reference passed its check.
        original=self.guard._segment_clearance
        calls=[]
        def collision(start,end,obstacles,inflation):
            calls.append(end)
            return -1 if end.x>0 and end.y>0 else original(start,end,obstacles,inflation)
        self.guard._segment_clearance=collision
        result=self.guard.plan_reference(self.snapshot,Pose(0,.3,1.2,0),2)
        self.assertFalse(result.publish_target)
        self.assertEqual(result.reason,'limited command swept segment blocked')
        self.assertGreaterEqual(len(calls),2)

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
