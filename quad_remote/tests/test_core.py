import unittest
from types import SimpleNamespace
from core import Latest, DriveCommand


class LatestTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.latest = Latest(clock=lambda: self.now)

    def drive(self, vx=.2, vy=0.0, wz=0.0):
        self.latest.set_drive(dict(vx=vx, vy=vy, wz=wz))

    def test_burst_retains_only_newest_value(self):
        for number in range(1, 401):
            self.drive(number / 1000)
        command, _ = self.latest.tick()
        self.assertAlmostEqual(command.vx, .4)
        self.assertEqual(self.latest.publish_count, 1)

    def test_idle_connection_never_publishes_or_takes_over(self):
        for _ in range(100):
            self.drive(0)
            self.assertEqual(self.latest.tick(), (None, None))

    def test_timeout_emits_zero_once_then_stops(self):
        self.drive(); self.latest.tick()
        self.now += 1.21
        self.assertEqual(self.latest.tick(), (DriveCommand(), None))
        self.assertEqual(self.latest.tick(), (None, None))

    def test_disconnect_clears_drive_and_unpublished_pose(self):
        self.drive(); self.latest.tick()
        self.latest.pose('stand'); self.latest.stop()
        self.assertEqual(self.latest.tick(), (DriveCommand(), None))

    def test_pose_stops_motion_and_is_sent_once(self):
        self.drive(); self.latest.tick(); self.latest.pose('crouch')
        self.assertEqual(self.latest.snapshot()['mode'], 'crouch')
        self.assertEqual(self.latest.tick(), (DriveCommand(), 'crouch'))
        self.assertEqual(self.latest.tick(), (None, None))

    def test_limits_and_invalid_numeric_input(self):
        self.drive(9, -9, 9)
        self.assertEqual(self.latest.tick()[0], DriveCommand(.8, -.8, 1.5))
        for value in (float('nan'), float('inf'), True, '1', None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.drive(value)

    def test_initial_mode_is_not_fabricated(self):
        self.assertEqual(self.latest.snapshot()['mode'], 'unknown')
        self.latest.pose('stand')
        self.assertEqual(self.latest.snapshot()['mode'], 'stand')

    def test_feedback_mapping_missing_values_and_staleness(self):
        self.latest.set_graph(True)
        self.latest.feedback('wheels', SimpleNamespace(name=['leg2_wheel', 'leg1_wheel'],
                            position=[2, 1], velocity=[.2, float('nan')], effort=[]))
        state = self.latest.snapshot(full=True)
        self.assertEqual([w['label'] for w in state['wheels']], ['FL', 'FR', 'RR', 'RL'])
        self.assertTrue(state['ros_connected'])
        self.now += 1.1
        self.assertTrue(self.latest.snapshot()['ros_connected'])

    def test_estop_clears_drive(self):
        self.drive(); self.latest.tick(); self.latest.set_mode('estop')
        self.assertEqual(self.latest.tick()[0], DriveCommand())

    def test_odom_snapshot_and_reset_flag(self):
        msg = SimpleNamespace(
            pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=1.2, y=-0.5, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
            twist=SimpleNamespace(twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.3, y=0.0, z=0.0),
                angular=SimpleNamespace(x=0.0, y=0.0, z=0.1))))
        self.latest.set_odom(msg)
        state = self.latest.snapshot()
        self.assertTrue(state['odom_ok'])
        self.assertAlmostEqual(state['x'], 1.2)
        self.assertAlmostEqual(state['dist'], 1.3, places=1)
        self.latest.request_odom_reset()
        self.assertTrue(self.latest.consume_odom_reset())
        self.assertFalse(self.latest.consume_odom_reset())


if __name__ == '__main__':
    unittest.main()
