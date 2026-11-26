import importlib
import math
import sys
import threading
import time
import types
import unittest
import tempfile
import csv
import json
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'cartpole_hardware'))
from cartpole_hardware.core import Geometry, guarded_velocity, parse_sensor

# Minimal ROS/serial seams allow interlock tests without a ROS installation.
serial = types.ModuleType('serial')
serial.Serial = object
sys.modules['serial'] = serial
for module in ('rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg',
               'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv', 'rcl_interfaces', 'rcl_interfaces.msg'):
    sys.modules[module] = types.ModuleType(module)
sys.modules['rclpy.node'].Node = object
sys.modules['rclpy.qos'].qos_profile_sensor_data = object()
class Message:
    def __init__(self, **kw):
        self.header = types.SimpleNamespace(stamp=None)
        self.__dict__.update(kw)
for module, names in [('sensor_msgs.msg', ['JointState']), ('std_msgs.msg', ['Float64', 'String']),
                      ('std_srvs.srv', ['Trigger', 'SetBool']), ('rcl_interfaces.msg', ['SetParametersResult'])]:
    for name in names:
        setattr(sys.modules[module], name, Message)
from cartpole_hardware.node import Cartpole, DEFAULTS
from cartpole_hardware.hybrid import PoleObserver
from cartpole_hardware.serial_io import Motor
from cartpole_hardware.analyze import analyze

class FakeMotor:
    def __init__(self):
        self.pos = 7500
        self.rpm = 0
        self.commands = []
        self.io = types.SimpleNamespace(reset_input_buffer=lambda: None)
    def number(self, cmd):
        return self.pos if cmd == 'POS' else self.rpm
    def velocity(self, rpm):
        self.commands.append(rpm)
    def stop(self, *args):
        self.commands.append('STOP')
        return True
    def enable(self):
        self.commands.append('ENABLE')


def rig():
    n = object.__new__(Cartpole)
    n.p = dict(DEFAULTS, hardware_configured=True)
    n.geo = Geometry(.04, 1, 3000, 1)
    n.lock = threading.Lock()
    n.sensor_error = n.limit_latched = ''
    n.sensor = (time.monotonic(), (1, 10000, 0, False, False, 1, 0))
    n.motor = FakeMotor()
    n.enabled = True
    n.homed = True
    n.pole_calibrated = True
    n.transport_failed = False
    n.mode = 'manual'
    n.fault = ''
    n.origin = n.pole_zero = 0
    n.x = n.v = n.theta = n.omega = n.output = 0.0
    n.command = .02
    n.command_time = time.monotonic()
    n.last_loop = time.monotonic() - .02
    n.last_pole = None
    n.hybrid = None
    n.hybrid_trial_id = 0
    n.hybrid_observer = PoleObserver()
    n.home_left_seen = n.home_left_released = False
    n.home_reference = 0.0
    n.home_start = time.monotonic()
    n.pub = types.SimpleNamespace(publish=lambda m: None)
    n.telemetry = types.SimpleNamespace(publish=lambda m: None)
    n.stop_confirmed = None
    n.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))
    n.get_logger = lambda: types.SimpleNamespace(error=lambda m: None)
    return n


class ControlTests(unittest.TestCase):
    def test_geometry_roundtrip_and_reduction(self):
        g = Geometry(.04, 10, 3000, -1)
        self.assertAlmostEqual(g.position(30000), -.04)
        self.assertAlmostEqual(g.rpm(.04), -600)

    def test_invalid_frames(self):
        good = 'CP1,2,5000,-23,0,1,1,0,1'
        self.assertTrue(parse_sensor(good)[4])
        for bad in ('garbage', good.replace(',0,1,1,', ',2,1,1,'), good + ',extra'):
            with self.assertRaises(ValueError):
                parse_sensor(bad)

    def test_braking_envelope(self):
        v = guarded_velocity(.5, .47, False, False, .025, .475, .5, .1, .005)
        self.assertAlmostEqual(v, 0, places=7)
        self.assertEqual(guarded_velocity(-.1, .2, True, False, 0, .5, .2, .1, .005), 0)
        with self.assertRaises(ValueError):
            guarded_velocity(float('nan'), 0, False, False, 0, 1, 1, 1, 0)

    def test_stale_sensor_disables(self):
        n = rig()
        n.sensor = (time.monotonic() - 1, n.sensor[1])
        n.tick()
        self.assertFalse(n.enabled)
        self.assertIn('timeout', n.fault)
        self.assertEqual(n.motor.commands[-1], 'STOP')

    def test_left_and_right_limits_latch(self):
        for left, right in ((True, False), (False, True), (True, True)):
            n = rig()
            n.sensor = (time.monotonic(), (2, 20000, 0, left, right, 1, 0))
            n.tick()
            self.assertFalse(n.enabled)
            self.assertFalse(n.homed)
            self.assertTrue(n.fault)

    def test_manual_and_command_watchdog(self):
        n = rig()
        n.tick()
        self.assertGreater(n.motor.commands[-1], 0)
        n.command_time = time.monotonic() - 1
        n.tick()
        self.assertFalse(n.enabled)
        self.assertIn('command timeout', n.fault)

    def test_velocity_topic_to_signed_serial_rpm(self):
        n = rig()
        n.geo = Geometry(.04, 9.7, 3000, 1)
        n.motor.pos = round(.0735 / n.geo.position(1))
        n.p.update(max_velocity_m_s=.1, max_motor_rpm=1500, max_acceleration_m_s2=1.0)
        drive = object.__new__(Motor)
        frames = []
        drive.io = types.SimpleNamespace(write=frames.append, read_until=lambda *args: b'OK\r\n')
        n.motor.velocity = drive.velocity
        for velocity, frame in ((.012, b'V175\r'), (-.012, b'V-175\r')):
            n.on_velocity(Message(data=velocity))
            n.last_loop = time.monotonic() - .025
            n.tick()
            self.assertEqual(frames[-1], frame)

    def test_homing_contact_stops_before_reversal(self):
        n = rig()
        n.mode = 'home_seek'
        n.output = -.01
        n.sensor = (time.monotonic(), (2, 20000, 0, True, False, 1, 0))
        n.tick()
        self.assertEqual(n.mode, 'home_stopping')
        self.assertEqual(n.output, 0)
        for _ in range(3):
            n.tick()
        self.assertEqual(n.mode, 'home_backoff')
        n.motor.pos += 75  # release 1 mm from contact
        n.sensor = (time.monotonic(), (3, 30000, 0, False, False, 1, 0))
        n.tick()
        self.assertEqual(n.x, 0)
        n.motor.pos += 2325  # travel another 31 mm into the valid range
        n.tick()
        self.assertTrue(n.homed)
        self.assertFalse(n.enabled)

    def test_homing_timeout(self):
        n = rig()
        n.mode = 'home_seek'
        n.home_start = time.monotonic() - 100
        n.tick()
        self.assertFalse(n.enabled)
        self.assertIn('homing timeout', n.fault)

    def test_motor_failure_disables_and_requires_restart(self):
        n = rig()
        n.motor.number = lambda c: (_ for _ in ()).throw(TimeoutError())
        n.tick()
        self.assertFalse(n.enabled)
        self.assertTrue(n.transport_failed)

    def test_balance_outside_window(self):
        n = rig()
        n.mode = 'balance'
        n.tick()  # count zero is DOWN -> +/-pi
        self.assertFalse(n.enabled)
        self.assertIn('capture window', n.fault)

    def test_configuration_gate(self):
        n = rig()
        n.p['hardware_configured'] = False
        with self.assertRaises(ValueError):
            n.ready()

    def test_serial_framing_rejections_and_stop(self):
        m = object.__new__(Motor)
        writes = []
        replies = iter([b'OK\r\n', b'-123\r\n', b'Unknown command\r\n', b'23'])
        m.io = types.SimpleNamespace(write=writes.append,
                                     read_until=lambda *args: next(replies))
        m.command('V10')
        self.assertEqual(writes[-1], b'V10\r')
        self.assertEqual(m.number('POS'), -123)
        with self.assertRaises(RuntimeError):
            m.command('EN')
        with self.assertRaises(TimeoutError):
            m.number('GN')
        m.emergency_zero()
        self.assertEqual(writes[-1], b'V0\r')


    def test_stop_verifies_stationary_window_before_disable(self):
        m = object.__new__(Motor)
        commands = []
        clock = [0.0]
        def advance(seconds):
            clock[0] += seconds
        m.command = commands.append
        m.number = lambda command: 250 if command == 'POS' else 0
        m.emergency_zero = lambda: commands.append('EMERGENCY_ZERO')
        with (patch('time.monotonic', side_effect=lambda: clock[0]),
              patch('time.monotonic_ns', side_effect=lambda: round(clock[0] * 1e9)),
              patch('time.sleep', side_effect=advance)):
            self.assertTrue(m.stop(timeout=1.0))
        self.assertGreaterEqual(round(clock[0] * 1e9), 60_000_000)
        self.assertEqual(commands, ['V0', 'DI'])

    def test_stop_failure_never_disables_drive(self):
        m = object.__new__(Motor)
        commands = []
        m.command = commands.append
        m.number = lambda command: (_ for _ in ()).throw(TimeoutError())
        m.emergency_zero = lambda: commands.append('RAW_V0')
        self.assertFalse(m.stop(timeout=.01))
        self.assertEqual(commands, ['V0', 'RAW_V0'])

    def test_unconfirmed_stop_blocks_rearming(self):
        n = rig()
        n.motor.stop = lambda *args: False
        n.disable()
        self.assertEqual(n.mode, 'stop_unconfirmed')
        self.assertTrue(n.transport_failed)
        self.assertFalse(n.homed)

    def test_sensor_only_publishes_without_motor(self):
        n = rig()
        n.enabled = False
        n.motor = None
        output = []
        n.telemetry.publish = output.append
        n.tick()
        self.assertEqual(len(output), 1)
        self.assertIn('"motor_available": false', output[0].data)
        with self.assertRaises(ValueError):
            n.ready()

    def test_configuration_200mm_center(self):
        n = rig()
        self.assertEqual(n.p['track_length_m'], .2)
        self.assertEqual(n.p['target_position_m'], .1)
        n.validate()



    def test_latched_transient_limit_stops_before_motor_poll(self):
        n = rig()
        n.limit_latched = 'right endstop event'
        n.motor.number = lambda c: self.fail('must stop before querying position')
        n.tick()
        self.assertFalse(n.enabled)
        self.assertIn('endstop event', n.fault)

    def test_recording_analysis(self):
        with tempfile.NamedTemporaryFile(mode='w+', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['receive_monotonic_ns','stream','payload_json'])
            for seq, blocked in [(1,False),(2,True),(4,False)]:
                writer.writerow([seq*5000000, 'sensor_raw', json.dumps(dict(
                    sequence=seq, board_us=seq*5000, host_monotonic_ns=seq*5000000,
                    left_blocked=blocked, right_blocked=False))])
            stream.flush()
            result = analyze(stream.name)
            self.assertEqual(result['observed_sensor_sequence_gaps'], 1)
            self.assertEqual(len(result['switch_transitions']), 2)
            self.assertEqual(result['board_interval_ms']['max'], 10)


if __name__ == '__main__':
    unittest.main()
