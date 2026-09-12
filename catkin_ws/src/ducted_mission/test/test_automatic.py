import importlib.util
import pathlib
import sys
import unittest
from dataclasses import replace

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src'))
SPEC=importlib.util.find_spec('ducted_mission.automatic')
if SPEC:
    from ducted_mission.automatic import AutomaticFlight, AutoConfig, Observation

class AutomaticTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(SPEC,'automatic flight sequence is not implemented')
        self.core=AutomaticFlight(AutoConfig(enabled=True,takeoff_rise=.8))
        self.o=Observation(healthy=True,armed=True,on_ground=True,controller='DISABLED',
                           mission='IDLE',x=0.,y=0.,z=.15,corridor_clear=True)

    def ack(self, action, now=0):
        self.core.ack(action.generation,action.command,True,'',now)

    def climb(self):
        action=self.core.start(self.o,0)[1];self.ack(action)
        self.o=replace(self.o,controller='HOLD',controller_ready=True,offboard=True)
        action=self.core.tick(self.o,.1)[0]
        self.assertEqual('takeoff',action.command)
        self.assertAlmostEqual(.95,action.z)
        self.ack(action,.1)
        self.o=replace(self.o,controller='TAKEOFF',on_ground=False,in_air=True,z=.5)
        self.assertFalse(self.core.tick(self.o,.2))
        self.o=replace(self.o,controller='HOLD',z=.95,speed=0.)
        action=self.core.tick(self.o,1)[0]
        self.assertEqual('mission_start',action.command)
        self.ack(action,1)
        self.o=replace(self.o,mission='RUNNING',mission_session='new-session')
        self.core.tick(self.o,1.1)

    def test_no_actions_on_launch(self):
        self.assertEqual((),self.core.tick(self.o,0))

    def test_disabled_and_unarmed_reject(self):
        self.assertFalse(AutomaticFlight(AutoConfig()).start(self.o,0)[0])
        self.assertFalse(self.core.start(replace(self.o,armed=False),0)[0])

    def test_obstacle_blocks_takeoff(self):
        self.assertFalse(self.core.start(replace(self.o,corridor_clear=False),0)[0])

    def test_full_sequence_holds_at_end(self):
        self.climb()
        self.o=replace(self.o,mission='SUCCEEDED')
        self.assertEqual((),self.core.tick(self.o,2))
        self.assertEqual('SUCCEEDED',self.core.state)

    def test_configured_landing_requires_ground_and_disarm(self):
        self.core=AutomaticFlight(AutoConfig(enabled=True,finish='land',takeoff_rise=.8))
        self.climb();self.o=replace(self.o,mission='SUCCEEDED')
        action=self.core.tick(self.o,2)[0];self.assertEqual('land',action.command);self.ack(action,2)
        self.o=replace(self.o,controller='LANDING',offboard=False)
        self.assertEqual((),self.core.tick(self.o,2.2))
        self.assertNotEqual('SUCCEEDED',self.core.state)
        self.o=replace(self.o,on_ground=True,in_air=False,armed=False,controller='DISABLED')
        self.core.tick(self.o,3);self.assertEqual('SUCCEEDED',self.core.state)

    def test_slow_landing_requests_controlled_descent_after_final_hold(self):
        self.core=AutomaticFlight(AutoConfig(enabled=True,finish='slow_land',takeoff_rise=.8))
        self.climb();self.o=replace(self.o,mission='SUCCEEDED')
        action=self.core.tick(self.o,2)[0]
        self.assertEqual('slow_land',action.command)
        self.ack(action,2)
        self.o=replace(self.o,controller='SLOW_DESCENT',z=.7)
        self.assertEqual((),self.core.tick(self.o,2.2))
        self.assertEqual('LANDING',self.core.state)

    def test_agl_takeoff_uses_measured_ground_with_nonzero_odom_origin(self):
        self.core=AutomaticFlight(AutoConfig(enabled=True,takeoff_agl=1.))
        self.o=replace(self.o,z=2.15,agl=.15)
        action=self.core.start(self.o,0)[1]
        self.assertIsNotNone(action)
        self.assertAlmostEqual(action.z,3.)

    def test_old_mission_success_does_not_finish_new_flight(self):
        self.o=replace(self.o,mission='SUCCEEDED',mission_session='old')
        self.climb()
        self.o=replace(self.o,mission='SUCCEEDED',mission_session='old')
        self.core.tick(self.o,2)
        self.assertNotEqual('SUCCEEDED',self.core.state)

    def test_cancel_during_engage_revokes_queued_takeoff(self):
        action=self.core.start(self.o,0)[1]
        stop=self.core.cancel(.1)
        self.ack(action,.2)
        self.assertFalse(self.core.dispatch_current(action))
        self.assertEqual('stop',stop[0].command)
        self.assertEqual('STOPPING',self.core.state)

    def test_rc_or_data_loss_aborts_without_auto_resume(self):
        self.climb()
        actions=self.core.tick(replace(self.o,healthy=False,reason='RC manual'),2)
        self.assertEqual('stop',actions[0].command)
        self.ack(actions[0],2)
        self.assertEqual('ABORTED',self.core.state)
        self.assertEqual((),self.core.tick(self.o,3))

    def test_climb_obstacle_aborts(self):
        a=self.core.start(self.o,0)[1];self.ack(a)
        self.o=replace(self.o,controller='HOLD',controller_ready=True,offboard=True,corridor_clear=False)
        self.assertEqual('stop',self.core.tick(self.o,.1)[0].command)

    def test_timeout_and_uncertain_rpc_latch_fault(self):
        a=self.core.start(self.o,0)[1]
        self.core.ack(a.generation,a.command,False,'outcome uncertain',1)
        self.assertEqual('FAULT',self.core.state)
        self.assertFalse(self.core.start(self.o,2)[0])

    def test_takeoff_requires_actual_in_air_and_arrival(self):
        a=self.core.start(self.o,0)[1];self.ack(a)
        self.o=replace(self.o,controller='HOLD',controller_ready=True,offboard=True)
        a=self.core.tick(self.o,.1)[0];self.ack(a,.1)
        self.assertEqual((),self.core.tick(self.o,1))

    def test_takeoff_height_must_enter_navigation_agl_envelope(self):
        self.assertFalse(self.core.start(replace(self.o,target_agl_safe=False),0)[0])

    def test_invalid_config(self):
        for change in ({'takeoff_rise':float('nan')},{'finish':'arm'},{'takeoff_rise':0}):
            with self.assertRaises(ValueError):AutoConfig(**change)

    def test_cloud_pose_pair_uses_history_not_latest_arrival(self):
        from ducted_mission import automatic
        self.assertTrue(hasattr(automatic,'matched_cloud_pose'))
        points=[(10.,(0.,0.,.4)),(10.18,(0.,0.,.45))]
        self.assertEqual((0.,0.,.4),automatic.matched_cloud_pose(10.,points))
        self.assertIsNone(automatic.matched_cloud_pose(9.,points))

    def test_arriving_at_takeoff_target_without_measured_height_cannot_start_mission(self):
        self.assertIn('height_measured',Observation.__dataclass_fields__)
        a=self.core.start(self.o,0)[1];self.ack(a)
        self.o=replace(self.o,controller='HOLD',controller_ready=True,offboard=True)
        a=self.core.tick(self.o,.1)[0];self.ack(a,.1)
        self.o=replace(self.o,controller='TAKEOFF',on_ground=False,in_air=True,z=.5,
                       height_measured=False)
        self.core.tick(self.o,.2)
        self.o=replace(self.o,controller='HOLD',z=.95,speed=0.)
        self.assertEqual((),self.core.tick(self.o,1))
        self.assertEqual('WAIT_TAKEOFF',self.core.state)

if __name__=='__main__':unittest.main()
