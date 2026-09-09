import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from test_scan_rejoin import RejoinTests
from scan_route_geometry import generate_world_serpentine

class ObstacleTests(unittest.TestCase):
    def setUp(self):
        self.fixture=RejoinTests(); self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.s=self.fixture.s; self.s.world_frame='world'
        self.s.skipped_waypoints=[]; self.s.consecutive_skips=0
        self.s.obstacle_resolution=.1

    def test_turns_never_reverse_x(self):
        route=generate_world_serpentine((0,0,1),width_m=6,length_m=10,sweep_count=5)
        previous=(0,0,1)
        for item in route:
            if item['kind']=='turn':
                self.assertEqual(item['world'][0],previous[0])
                self.assertGreater(item['world'][1],previous[1])
            previous=item['world']
        self.assertEqual(route[-1]['world'],(6,10,1))

    def test_occupied_requires_fresh_world_cloud(self):
        s=self.s; cloud=NS(header=NS(frame_id='world'))
        s.latest_obstacle_cloud=(cloud,100)
        ns=s.goal_occupied.__globals__
        with patch.dict(ns,point_cloud2=NS(read_points=lambda *a,**kw:iter([(1,0,1)]))):
            self.assertTrue(s.goal_occupied((1,0,1)))
            self.assertFalse(s.goal_occupied((2,0,1)))
            self.fixture.now.return_value=102
            self.assertFalse(s.goal_occupied((1,0,1)))
            self.fixture.now.return_value=100; cloud.header.frame_id='camera'
            self.assertFalse(s.goal_occupied((1,0,1)))

    def test_skip_waits_for_cancellation_and_settle(self):
        s=self.s; s.begin_obstacle_skip(); s.client.simple_state=1
        s.advance_obstacle_skip(); self.assertEqual(s.route_index,0)
        s.client.simple_state=2; s.advance_obstacle_skip()
        self.fixture.now.return_value=101; s.dispatch_route_leg=Mock()
        s.advance_obstacle_skip()
        self.assertEqual(s.route_index,1); self.assertEqual(s.skipped_waypoints,[0])
        s.dispatch_route_leg.assert_called_once()

    def test_consecutive_skip_limit_stops(self):
        s=self.s; s.consecutive_skips=3; s.begin_obstacle_skip()
        self.assertIsNone(s.active)
        self.assertEqual(s.publish_status.call_args.args[0],'FAILED')

    def test_stop_during_skip_prevents_next_goal(self):
        s=self.s; s.begin_obstacle_skip(); s.on_cancel(None)
        count=s.client.send_goal.call_count; s.on_timer(None)
        self.assertEqual(s.client.send_goal.call_count,count)

if __name__=='__main__': unittest.main()
