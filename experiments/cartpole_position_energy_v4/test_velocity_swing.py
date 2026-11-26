"""Offline controller and runner tests, with a coherent monotonic ROS clock."""
import math
import unittest
from unittest.mock import Mock, patch

from run_velocity import Hardware, healthy, ready_to_start, trial
from velocity_swing import SCALE, Settings, Swing

PERIOD = 20_000_000


def sample(stamp, phi=0, rate=0, x=0.0735, **fields):
    theta = math.radians(phi) % math.tau - math.pi  # phi=0 is down; theta=0 is up
    return dict(host_monotonic_ns=stamp, cart_m=x, cart_m_s=0.0, pole_rad=theta, pole_rad_s=rate,
                sensor_age_s=0.005, fault="", homed=True, pole_calibrated=True,
                motor_available=True, command_m_s=0.0, mode="manual") | fields


def status():
    return dict(mode="disabled", homed=True, pole_calibrated=True, motor_available=True,
                sensor_fresh=True, fault="", sensor_error="", limit_latched="",
                left_blocked=False, right_blocked=False, enabled=False, stop_confirmed=True)


def reference():
    return dict(boot_id="pi", captured_ns=1_000_000_000, cart_m=0.0735,
                counts_per_rev=3000, gear_ratio=9.7, pulley_m=0.04)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Settings(fraction=0.3044, speed=0.08, acceleration=0.5)
        self.swing = Swing(0.0735, 0, self.cfg)
        self.stamp = 0

    def tick(self, phi=0, rate=0, x=0.0735, speed=0.0):
        self.stamp += PERIOD
        return self.swing.update(self.stamp, sample(self.stamp, phi, rate, x, cart_m_s=speed))

    def test_velocity_limits_and_braking_as_stroke_runs_out(self):
        v, decision = self.tick()
        self.assertEqual(v, 0)
        self.assertAlmostEqual(decision["stroke_end_m"], 0.0735 + 11000*SCALE)
        for _ in range(8):
            new, _ = self.tick()
            self.assertLessEqual(abs(v-new), 0.010001)
            v = new
        self.assertGreater(v, 0.02)
        self.assertLessEqual(v, self.cfg.speed)
        for _ in range(16):
            v, _ = self.tick(x=decision["stroke_end_m"], speed=0.05)
        for _ in range(8):
            v, _ = self.tick(x=decision["stroke_end_m"])
        self.assertEqual(v, 0)

    def test_final_stroke_uses_remaining_distance_and_terminates_on_return(self):
        for _ in range(10):
            self.tick(phi=15, speed=0.04)
        x = 0.081
        _, decision = self.tick(phi=-1, rate=-25, x=x, speed=-0.01)
        self.assertEqual(decision["direction"], -1)
        self.assertEqual(decision["reason"], "reduced_final_push")
        estimated = 0.0735 + 9 * 0.04 * 0.02 + (0.04 - 0.01) * 0.01
        self.assertAlmostEqual(decision["estimated_cart_m"], estimated)
        expected = estimated + 0.3044 * (0.0735 - 9000*SCALE - estimated)
        self.assertAlmostEqual(decision["stroke_end_m"], expected, delta=2*SCALE)
        for _ in range(12):
            v, _ = self.tick(phi=-15, x=x)
        self.assertLess(v, 0)
        self.assertEqual(self.tick(phi=1, rate=12, x=x)[0], 0)
        self.assertEqual(self.swing.outcome, "final_push_returned_without_upright_entry")

    def test_rail_guard_after_slew_in_both_directions(self):
        for _ in range(6):
            self.tick(phi=15)
        self.assertEqual(self.tick(phi=15, x=self.swing.upper - 0.001)[0], 0)
        for _ in range(5):
            self.tick(phi=15)
        self.tick(phi=-1, rate=-12)
        for _ in range(12):
            self.tick(phi=-15)
        self.assertLess(self.swing.last_request, 0)
        self.assertEqual(self.tick(phi=-15, x=self.swing.lower + 0.001)[0], 0)
        with self.assertRaisesRegex(RuntimeError, "travel window"):
            self.tick(x=self.swing.upper + 0.001)

    def test_upright_entry_stops_and_records_entry_rate(self):
        self.tick()
        v, _ = self.tick(phi=-169, rate=-4)
        self.assertEqual((v, self.swing.outcome), (0, "upright_entry"))
        self.assertAlmostEqual(math.degrees(self.swing.closest), 11)
        self.assertEqual(self.swing.entry["pole_rate_rad_s"], -4)

    def test_integrates_signed_speed_and_actual_sample_interval(self):
        self.tick(x=0.080, speed=0.03)
        stamp = self.stamp + 35_000_000
        self.swing.update(stamp, sample(stamp, x=0.081, cart_m_s=0.05))
        self.assertAlmostEqual(self.swing.estimated_x, 0.0735 + 0.04 * 0.035)
        self.swing.update(stamp + PERIOD, sample(stamp + PERIOD, x=0.081, cart_m_s=-0.05))
        self.swing.update(stamp + 2 * PERIOD, sample(stamp + 2 * PERIOD, x=0.081, cart_m_s=-0.05))
        self.assertAlmostEqual(self.swing.estimated_x, 0.0735 + 0.04 * 0.035 - 0.05 * 0.02)

    def test_requested_limits_and_70_mm_window(self):
        settings = Settings(speed=0.275, acceleration=3.5)
        for centre in (0.070, 0.0735, 0.077):
            swing = Swing(centre, 0, settings)
            self.assertAlmostEqual(swing.lower, centre - 0.035)
            self.assertAlmostEqual(swing.upper, centre + 0.035)

    def test_startup_turnaround_before_five_degree_crossing_arms(self):
        for phi, rate in [(0, 0), (1, .5), (2, .5), (3.4, .3)] + [(3.5, 0)] * 6:
            self.tick(phi=phi, rate=rate, speed=0.04)
        _, decision = self.tick(phi=2.8, rate=-0.3, speed=0.04)
        self.assertEqual(decision["reason"], "startup_turnaround")
        self.assertEqual(decision["direction"], -1)
        self.assertEqual(self.swing.strokes, 2)
        for _ in range(8):
            request, _ = self.tick(phi=2, rate=-0.3)
        self.assertLess(request, 0)

    def test_phase_advance_issues_one_stroke_before_crossing(self):
        from energy_core import INERTIA, MG
        for energy, should_advance in ((0.72, True), (0.85, False)):
            with self.subTest(energy=energy):
                self.setUp()
                self.tick(phi=-20)
                self.swing.strokes = 3
                for _ in range(8):
                    self.tick(phi=-20)
                self.swing.phase.last_target = -1
                rate = math.sqrt(4 * MG * energy / INERTIA)
                _, decision = self.tick(phi=-12, rate=rate)
                if should_advance:
                    self.assertEqual(decision["reason"], "phase_advance")
                    self.assertLessEqual(decision["predicted_crossing_s"], 0.06)
                    self.assertIsNone(self.tick(phi=1, rate=rate)[1])
                else:
                    self.assertIsNone(decision)
                    self.assertEqual(self.tick(phi=1, rate=rate)[1]["reason"], "reduced_final_push")

    def test_measured_overspeed_aborts(self):
        with self.assertRaisesRegex(RuntimeError, "RPM limit"):
            self.swing.update(PERIOD, sample(PERIOD, cart_m_s=0.28))

    def test_stalled_pole_aborts(self):
        with self.assertRaisesRegex(RuntimeError, "stalled"):
            for _ in range(110):
                self.tick()

    def test_missing_feedback_aborts(self):
        self.tick()
        with self.assertRaisesRegex(RuntimeError, "Stale"):
            self.swing.update(5*PERIOD+1, sample(PERIOD))
        with self.assertRaisesRegex(RuntimeError, "interval"):
            self.swing.update(10*PERIOD, sample(10*PERIOD))
        with self.assertRaisesRegex(RuntimeError, "stalled"):
            for _ in range(110):
                self.tick()

    def test_reject_out_of_range_parameters(self):
        for field, value in (("speed", 0.276), ("acceleration", 3.51), ("energy", 0),
                             ("fraction", float("nan")), ("duration", 21), ("speed", -0.01)):
            with self.subTest(field=field), self.assertRaises(ValueError):
                Settings(**{field: value})


class RunnerTests(unittest.TestCase):
    def test_preflight_rejects_endstops_stale_or_unsettled_state(self):
        s, q, ref = status(), sample(PERIOD, mode="disabled"), reference()
        healthy(s, q, PERIOD, PERIOD)
        self.assertTrue(ready_to_start(s, q, ref))
        for key, value in (("fault", "fault"), ("left_blocked", True), ("sensor_fresh", False)):
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                healthy(s | {key: value}, q, PERIOD, PERIOD)
        with self.assertRaises(RuntimeError):
            healthy(s, q, PERIOD+90_000_000, PERIOD)
        with self.assertRaises(RuntimeError):
            healthy(s, q | {"pole_rad_s": float("nan")}, PERIOD, PERIOD)
        for changed in ({"cart_m_s": 0.02}, {"cart_m": 0.080}, {"pole_rad": 0.0}, {"mode": "manual"}):
            self.assertFalse(ready_to_start(s, q | changed, ref))

    def run_trial(self, hw, execute=True):
        summary, events = dict(completed=False, motion_attempted=False, stop_confirmed=None), []
        self.summary = summary
        with patch("time.monotonic_ns", return_value=1_500_000_000):
            trial(hw, Settings(), execute, reference(), "pi", summary, events)
        return summary

    def test_check_only_never_enables_or_commands(self):
        hw = Mock()
        summary = self.run_trial(hw, False)
        self.assertTrue(summary["completed"])
        self.assertFalse(summary["motion_attempted"])
        hw.call.assert_not_called()
        hw.send.assert_not_called()

    def test_invalid_reference_blocks_enable(self):
        hw = Mock()
        with self.assertRaisesRegex(ValueError, "older"):
            with patch("time.monotonic_ns", return_value=122_000_000_000):
                trial(hw, Settings(), True, reference(), "pi", {}, [])
        hw.call.assert_not_called()

    def test_stop_after_enable_timeout_and_runtime_failure(self):
        for enable_failure in (True, False):
            hw = Mock()
            if enable_failure:
                hw.call.side_effect = [RuntimeError("enable timeout"), None]
            else:
                hw.run.side_effect = RuntimeError("telemetry lost")
            with self.subTest(enable_failure=enable_failure), self.assertRaises(RuntimeError):
                self.run_trial(hw)
            hw.send.assert_called_once_with(0.0)
            self.assertEqual([c.args[0] for c in hw.call.call_args_list], ["enable", "stop"])
            self.assertTrue(self.summary["stop_confirmed"])
            self.assertFalse(self.summary["completed"])

    def test_failed_stop_propagates(self):
        hw = Mock()
        hw.call.side_effect = [None, RuntimeError("stop timeout")]
        with patch("builtins.print"), self.assertRaisesRegex(RuntimeError, "stop timeout"):
            self.run_trial(hw)
        self.assertIs(self.summary["stop_confirmed"], False)
        self.assertFalse(self.summary["completed"])

    def test_client_publishes_signed_speed_and_logs_feedback(self):
        hw = object.__new__(Hardware)
        hw.node, hw.send = Mock(), Mock()
        hw.node.count_publishers.return_value = 1
        clock = [1_000_000_000]
        hw.q = sample(clock[0], mode="disabled")
        hw.status = status()
        phases = iter([15]*10 + [-1] + [-15]*12 + [-170])
        def spin():
            clock[0] += PERIOD
            hw.q = sample(clock[0], next(phases), rate=-10)
            hw.status_at = clock[0]
        hw.spin = spin
        summary, events = {}, []
        with patch("time.monotonic_ns", side_effect=lambda: clock[0]):
            hw.run(Settings(), summary, events)
        commands = [call.args[0] for call in hw.send.call_args_list]
        self.assertLess(min(commands), 0)
        self.assertGreater(max(commands), 0)
        self.assertEqual(summary["outcome"], "upright_entry")
        self.assertTrue(summary["upright_entered"])
        self.assertEqual(events[-1]["requested_m_s"], 0.0)
        self.assertIn("applied_m_s", events[-1])
        self.assertIn("measured_m_s", events[-1])


if __name__ == "__main__":
    unittest.main()
