"""No ROS connections: exercise actual rejoin methods with mocked action client."""
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from test_field_repairs import FieldRepairs, Scan

class RejoinTests(unittest.TestCase):
    def setUp(self):
        self.s = FieldRepairs().scan()
        s=self.s; s.route_origin=(0,0,1); s.stop_velocity_tolerance=.15
        s.stop_settle_time=.6; s.post_orbit_cooldown=5
        self.odom=s.require_flight_ready.return_value
        self.odom.pose.pose.orientation=NS(x=0,y=0,z=0,w=1)
        self.odom.twist=NS(twist=NS(linear=NS(x=0,y=0,z=0),angular=NS(z=0)))
        self.set_position(.4,.2)
        s.capture_rejoin(self.odom); s.rejoin['started']=100
        self.clock=patch('time.monotonic',return_value=100); self.now=self.clock.start()
        self.addCleanup(self.clock.stop)

    def set_position(self,x,y):
        self.odom.pose.pose.position.x=x; self.odom.pose.pose.position.y=y

    def test_projection_and_segmented_return_freeze_route(self):
        s=self.s; route=list(s.route)
        self.assertEqual(s.rejoin['anchor'],(.4,0,1))
        self.set_position(4,3); s.dispatch_rejoin()
        goal=s.client.send_goal.call_args.args[0]
        self.assertAlmostEqual(math.dist((4,3,1),(goal.center.x,goal.center.y,goal.center.z)),1.8)
        self.assertEqual(s.route,route); self.assertEqual(s.route_index,0)
        self.assertEqual(s.phase,'REJOINING')
        s.class_ready=True
        # No target fields needed: detection callback must return immediately.
        s.on_raw_target(NS())

    def test_alignment_then_stability_then_resume(self):
        s=self.s; self.set_position(.4,0)
        self.odom.pose.pose.orientation.z=math.sin(.5)
        self.odom.pose.pose.orientation.w=math.cos(.5)
        s.dispatch_rejoin(); s.advance_rejoin()
        self.now.return_value=101; s.advance_rejoin()
        self.assertEqual(s.phase,'REJOIN_ALIGN')
        goal=s.client.send_goal.call_args.args[0]
        self.assertEqual(goal.skill,'ROTATE'); self.assertEqual(goal.direction,'right')
        self.assertAlmostEqual(goal.angle,1.0)
        self.odom.pose.pose.orientation.z=0; self.odom.pose.pose.orientation.w=1
        s.client.simple_state=2; s.client.get_state.return_value=3
        s.client.get_result.return_value=NS(success=True)
        s.advance_rejoin(); self.now.return_value=102
        s.dispatch_route_leg=Mock()
        ns=Scan.advance_rejoin.__globals__
        with patch.object(ns['rospy'],'Duration',lambda x:x,create=True): s.advance_rejoin()
        s.dispatch_route_leg.assert_called_once()
        self.assertIsNone(s.rejoin); self.assertEqual(s.route_index,0)

    def test_drift_fails_without_resuming(self):
        s=self.s; self.set_position(.4,0); s.dispatch_rejoin()
        self.set_position(.8,0); s.advance_rejoin()
        self.assertIsNone(s.active)
        self.assertEqual(s.publish_status.call_args.args[0],'FAILED')

    def test_cancel_in_return_never_resumes(self):
        s=self.s; self.set_position(3,3); s.dispatch_rejoin()
        s.on_cancel(None); count=s.client.send_goal.call_count
        s.on_timer(None)
        self.assertIsNone(s.rejoin); self.assertEqual(s.client.send_goal.call_count,count)

    def test_reverse_segment_yaw_is_preserved(self):
        s=self.s; s.route=[dict(world=(6,0,1)),dict(world=(5,0,1))]; s.route_index=1
        self.set_position(5.6,.4); s.capture_rejoin(self.odom)
        self.assertEqual(s.rejoin['anchor'],(5.6,0,1))
        self.assertAlmostEqual(abs(s.rejoin['yaw']),math.pi)

if __name__=='__main__': unittest.main()
