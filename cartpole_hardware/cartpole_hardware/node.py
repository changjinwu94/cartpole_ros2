import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger, SetBool
import serial

from .core import Geometry, finite, guarded_velocity, parse_sensor, wrap
from .serial_io import Motor
from rcl_interfaces.msg import SetParametersResult
from .gain_bank import CANDIDATES
from .hybrid import HybridController, State, DESIGN_DT, PoleObserver


DEFAULTS = dict(
    arduino_port='/dev/cartpole_arduino', motor_port='/dev/cartpole_motor',
    arduino_baud=115200, motor_baud=9600, serial_timeout_s=0.03,
    hardware_configured=False, motor_connected=False, motor_counts_per_rev=3000,
    pulley_circumference_m=0.04, motor_revs_per_pulley_rev=1.0,
    motor_sign=1, pole_sign=1, pole_ppr=360,
    track_length_m=0.2, soft_margin_m=0.025, stop_margin_m=0.005,
    max_velocity_m_s=0.05, max_motor_rpm=300.0,
    braking_deceleration_m_s2=0.1, max_acceleration_m_s2=0.1,
    home_velocity_m_s=0.01, home_timeout_s=60.0,
    sensor_timeout_s=0.1, command_timeout_s=0.25, loop_hz=50.0,
    stop_timeout_s=1.0, stationary_rpm=5.0,
    hybrid_candidate_id=0, hybrid_enabled=False, hybrid_swing_timeout_s=15.0, hybrid_hold_s=5.0,
    balance_enabled=False, balance_gains=[0.0, 0.0, 0.0, 0.0],
    balance_angle_limit_rad=0.25, target_position_m=0.1,
)


class Cartpole(Node):
    def __init__(self):
        super().__init__('cartpole')
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in DEFAULTS}
        self.validate()
        self.geo = Geometry(self.p['pulley_circumference_m'],
                            self.p['motor_revs_per_pulley_rev'],
                            self.p['motor_counts_per_rev'], self.p['motor_sign'])
        self.lock = threading.Lock()
        self.sensor = None
        self.sensor_error = ''
        self.limit_latched = ''
        self.running = True
        self.arduino = None
        self.motor = None
        self.enabled = self.homed = self.pole_calibrated = False
        self.mode = 'disabled'
        self.fault = ''
        self.add_on_set_parameters_callback(self.on_runtime_parameters)
        self.transport_failed = False
        self.origin = 0.0
        self.pole_zero = 0
        self.x = self.v = self.theta = self.omega = self.output = 0.0
        self.command = 0.0
        self.command_time = -math.inf
        self.last_loop = time.monotonic()
        self.last_pole = None
        self.hybrid = None
        self.hybrid_trial_id = 0
        self.hybrid_observer = PoleObserver()
        self.hybrid_state = None
        self.state_at = -math.inf
        self.home_start = 0.0
        self.home_left_seen = False
        self.home_left_released = False
        self.home_reference = 0.0
        self.home_last_position = 0.0
        self.home_stationary = 0
        self.home_stop_start = 0.0
        self.stop_confirmed = None
        self.sensor_packets = self.sensor_bad_frames = self.sensor_dropped = 0
        self.loop_dt_s = self.motor_roundtrip_s = 0.0
        self.sensor_raw = self.create_publisher(String, 'sensor_raw', qos_profile_sensor_data)
        self.telemetry = self.create_publisher(String, 'telemetry', qos_profile_sensor_data)
        self.pub = self.create_publisher(JointState, 'joint_states', qos_profile_sensor_data)
        self.status = self.create_publisher(String, 'status', 10)
        self.create_subscription(Float64, 'cmd_velocity', self.on_velocity, 1)
        self.create_service(SetBool, 'enable', self.on_enable)
        self.create_service(SetBool, 'balance', self.on_balance)
        self.create_service(Trigger, 'home', self.on_home)
        self.create_service(Trigger, 'zero_pole_down', self.on_zero)
        self.create_service(Trigger, 'stop', self.on_stop)
        self.create_service(Trigger, 'reset_fault', self.on_reset)
        self.create_service(SetBool, 'swing_balance', self.on_swing_balance)
        self.create_service(SetBool, 'lqr_balance', self.on_lqr_balance)
        try:
            self.arduino = serial.Serial(self.p['arduino_port'], self.p['arduino_baud'],
                                         timeout=0.05, exclusive=True)
            self.arduino.reset_input_buffer()
            if self.p['motor_connected']:
                self.motor = Motor(self.p['motor_port'], self.p['motor_baud'],
                                   self.p['serial_timeout_s'])
            self.thread = threading.Thread(target=self.read_sensors, daemon=True)
            self.thread.start()
        except Exception:
            self.close()
            raise
        self.control_timer = self.create_timer(1 / self.p['loop_hz'], self.tick)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info('Disabled. Configure geometry, calibrate pole, then home.')

    def on_runtime_parameters(self, parameters):
        # One immutable candidate can be selected only between stopped trials.
        if (len(parameters)!=1 or parameters[0].name!='hybrid_candidate_id'
                or type(parameters[0].value) is not int):
            return SetParametersResult(successful=False,
                reason='Only integer hybrid_candidate_id can change at runtime; restart for configuration')
        candidate=parameters[0].value
        if self.enabled or self.fault or not self.stop_confirmed:
            return SetParametersResult(successful=False,reason='Disabled, fault-free confirmed stop required')
        if not 0<=candidate<len(CANDIDATES):
            return SetParametersResult(successful=False,reason='Unknown screened LQR candidate')
        self.p['hybrid_candidate_id']=candidate
        return SetParametersResult(successful=True)

    def validate(self):
        p = self.p
        if type(p['hybrid_candidate_id']) is not int or not 0<=p['hybrid_candidate_id']<len(CANDIDATES):
            raise ValueError('Unknown hybrid LQR candidate')
        for key, value in p.items():
            if isinstance(value, float):
                finite(value)
        for key in ('serial_timeout_s', 'pulley_circumference_m',
                    'motor_revs_per_pulley_rev', 'motor_counts_per_rev', 'pole_ppr',
                    'track_length_m', 'max_velocity_m_s', 'max_motor_rpm',
                    'braking_deceleration_m_s2', 'max_acceleration_m_s2',
                    'home_velocity_m_s', 'home_timeout_s', 'sensor_timeout_s',
                    'command_timeout_s', 'loop_hz', 'balance_angle_limit_rad',
                    'stop_timeout_s', 'stationary_rpm'):
            if p[key] <= 0:
                raise ValueError(key + ' must be positive')
        if p['pole_sign'] not in (-1, 1) or p['motor_sign'] not in (-1, 1):
            raise ValueError('direction signs must be +/-1')
        if not 0 < p['stop_margin_m'] < p['soft_margin_m'] < p['track_length_m'] / 2:
            raise ValueError('invalid travel margins')
        if not p['soft_margin_m'] < p['target_position_m'] < p['track_length_m'] - p['soft_margin_m']:
            raise ValueError('target outside soft limits')
        if p['home_velocity_m_s'] > p['max_velocity_m_s']:
            raise ValueError('home speed exceeds maximum')
        if len(p['balance_gains']) != 4:
            raise ValueError('four balance gains required')
        for gain in p['balance_gains']:
            finite(gain)
        if p['loop_hz'] > 100:
            raise ValueError('loop rate above supported commissioning range')

    def read_sensors(self):
        previous = None
        while self.running:
            try:
                raw = self.arduino.read_until(b'\n', 160)
                if not raw:
                    continue
                # Partial/malformed packets never refresh freshness.
                if not raw.endswith(b'\n'):
                    self.sensor_bad_frames += 1
                    continue
                try:
                    frame = parse_sensor(raw.decode('ascii'))
                except (ValueError, UnicodeError):
                    self.sensor_bad_frames += 1
                    continue
                seq, us, count, left, right, z, bad = frame
                if previous is not None:
                    delta = (seq - previous) & 0xffffffff
                    if delta == 0:
                        continue
                    if 1 < delta <= 0x7fffffff:
                        self.sensor_dropped += delta - 1
                    if delta > 0x7fffffff:
                        raise RuntimeError('Arduino reset / out-of-order frame; restart node')
                if bad > 0:
                    raise RuntimeError('illegal encoder transition; inspect wiring and restart')
                previous = seq
                with self.lock:
                    self.sensor = (time.monotonic(), frame)
                    if self.enabled:
                        if right:
                            self.limit_latched = 'right endstop event'
                        elif left and self.mode.startswith('home_'):
                            self.home_left_seen = True
                            if self.home_left_released:
                                self.limit_latched = 'left endstop reactivated during backoff'
                        elif left:
                            self.limit_latched = 'left endstop event'
                self.sensor_packets += 1
                self.sensor_raw.publish(String(data=json.dumps(dict(
                    host_monotonic_ns=time.monotonic_ns(), sequence=seq, board_us=us,
                    count=count, left_blocked=left, right_blocked=right, index_raw=z,
                    invalid_transitions=bad, dropped_frames=self.sensor_dropped,
                    bad_frames=self.sensor_bad_frames))))
            except Exception as exc:
                with self.lock:
                    self.sensor_error = str(exc)
                return

    def snapshot(self):
        with self.lock:
            return self.sensor, self.sensor_error

    def fresh(self):
        sample, error = self.snapshot()
        return bool(sample and not error and
                    time.monotonic() - sample[0] <= self.p['sensor_timeout_s'])

    def disable(self):
        if self.hybrid and self.hybrid.mode in ('swing', 'balance'):
            self.hybrid.abort(self.fault or 'operator_stop')
        self.enabled = False
        self.mode = 'disabled'
        self.output = self.command = 0.0
        self.command_time = -math.inf
        if self.motor:
            self.stop_confirmed = self.motor.stop(self.p['stop_timeout_s'],
                                                  self.p['stationary_rpm'])
            if not self.stop_confirmed:
                self.mode = 'stop_unconfirmed'
                self.fault = self.fault or 'stop not confirmed; use physical E-stop and inspect'
                self.transport_failed = True
                self.homed = False
                self.get_logger().error('Stop NOT confirmed; drive disable withheld')
        return self.stop_confirmed

    def fail(self, reason, transport=False):
        if not self.fault:
            self.get_logger().error(reason)
        self.fault = reason
        self.transport_failed |= transport
        self.homed = False
        self.disable()

    def ready(self):
        if self.motor is None:
            raise ValueError('sensor-only mode: motor_connected=false')
        if not self.p['hardware_configured']:
            raise ValueError('set hardware_configured only after commissioning checklist')
        if self.fault or self.transport_failed:
            raise ValueError('fault: ' + self.fault)
        if not self.fresh():
            raise ValueError('sensor stream is stale')

    def enable_drive(self):
        with self.lock:
            self.limit_latched = ''
        self.motor.enable()
        self.enabled = True
        self.stop_confirmed = None
        self.output = self.command = 0.0
        self.command_time = -math.inf

    def response(self, response, action):
        try:
            action()
            response.success, response.message = True, 'OK'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        except Exception as exc:
            self.fail('serial operation failed: ' + str(exc), transport=True)
            response.success, response.message = False, self.fault
        return response

    def on_velocity(self, message):
        try:
            value = finite(message.data)
            if self.enabled and self.mode == 'manual':
                self.command = value
                self.command_time = time.monotonic()
        except ValueError:
            self.fail('non-finite velocity command')

    def on_enable(self, request, response):
        def action():
            if not request.data:
                if self.disable() is False:
                    raise ValueError('stop NOT confirmed; use physical E-stop')
                return
            self.ready()
            if self.enabled:
                raise ValueError('already enabled; stop before changing mode')
            sample, _ = self.snapshot()
            if not self.homed or sample[1][3] or sample[1][4]:
                raise ValueError('home first and clear both endstops')
            self.enable_drive()
            self.mode = 'manual'
        return self.response(response, action)

    def on_home(self, request, response):
        def action():
            self.ready()
            if self.enabled:
                raise ValueError('stop before homing')
            sample, _ = self.snapshot()
            if sample[1][4]:
                raise ValueError('right endstop active; inspect/reposition manually')
            if sample[1][3]:
                raise ValueError('clear left switch before homing')
            self.homed = False
            with self.lock:
                self.home_left_seen = False
                self.home_left_released = False
                self.mode = 'home_seek'
            self.home_reference = self.x
            self.enable_drive()
            self.home_start = time.monotonic()
        return self.response(response, action)

    def on_zero(self, request, response):
        def action():
            if self.enabled or not self.fresh():
                raise ValueError('disable drive and provide fresh sensor data')
            sample, _ = self.snapshot()
            self.pole_zero = sample[1][2]
            self.pole_calibrated = True
            self.last_pole = None
            self.hybrid_observer = PoleObserver()
            self.hybrid_state = None
        return self.response(response, action)

    def on_balance(self, request, response):
        def action():
            if not request.data:
                if self.disable() is False:
                    raise ValueError('stop NOT confirmed; use physical E-stop')
                return
            self.ready()
            if self.enabled:
                raise ValueError('stop before starting balance')
            sample, _ = self.snapshot()
            if (not self.homed or not self.pole_calibrated or sample[1][3] or sample[1][4]
                    or abs(self.theta) > self.p['balance_angle_limit_rad']):
                raise ValueError('home, calibrate and hold pole near upright with clear switches')
            if not self.p['balance_enabled'] or not any(self.p['balance_gains']):
                raise ValueError('balance disabled or gains not configured')
            self.enable_drive()
            self.mode = 'balance'
        return self.response(response, action)

    def on_swing_balance(self, request, response):
        return self.start_hybrid(request, response, balance_only=False)

    def on_lqr_balance(self, request, response):
        return self.start_hybrid(request, response, balance_only=True)

    def start_hybrid(self, request, response, balance_only):
        def action():
            if not request.data:
                if self.disable() is False:
                    raise ValueError('stop NOT confirmed; use physical E-stop')
                return
            self.ready()
            if not self.p['hybrid_enabled']:
                raise ValueError('Hybrid disabled; use reviewed LQR config after commissioning')
            if self.enabled or not self.homed or not self.pole_calibrated:
                raise ValueError('Prepare home/calibration/centre and stop first')
            sample, _ = self.snapshot()
            if sample[1][3] or sample[1][4] or not self.stop_confirmed:
                raise ValueError('Clear switches and confirmed stop required')
            if time.monotonic()-self.state_at > .035:
                raise ValueError('No state estimate within 35ms')
            if abs(1/self.p['loop_hz']-DESIGN_DT)>1e-9:
                raise ValueError('LQR gain requires 50Hz loop')
            # Never expand the configured hardware limits to satisfy the model.
            if (abs(self.p['track_length_m']-.147)>1e-8 or
                    self.p['motor_sign']!=1 or self.p['pole_sign']!=1 or
                    self.p['pole_ppr']!=500 or
                    self.p['motor_counts_per_rev']!=3000 or
                    abs(self.p['motor_revs_per_pulley_rev']-9.7)>1e-8 or
                    abs(self.p['pulley_circumference_m']-.04)>1e-8 or
                    abs(self.p['target_position_m']-.0735)>1e-8):
                raise ValueError('LQR model geometry/sign configuration mismatch')
            controller=HybridController(target=self.p['target_position_m'],
                lower=max(.050,self.p['soft_margin_m']),
                upper=min(.100,self.p['track_length_m']-self.p['soft_margin_m']),
                vmax=min(.1,self.p['max_velocity_m_s'],
                    self.p['max_motor_rpm']*self.p['pulley_circumference_m']/
                    (60*self.p['motor_revs_per_pulley_rev'])),
                amax=min(1.,self.p['max_acceleration_m_s2']),
                braking=self.p['braking_deceleration_m_s2'],
                margin=self.p['stop_margin_m'],
                swing_timeout=self.p['hybrid_swing_timeout_s'],
                hold_seconds=self.p['hybrid_hold_s'],
                gains=CANDIDATES[self.p['hybrid_candidate_id']]['gains'])
            if self.hybrid_observer.samples<3 or self.hybrid_state is None:
                raise ValueError('Wait for hybrid observer to warm up after calibration')
            controller.start(self.hybrid_state,balance_only)
            self.hybrid=controller
            self.enable_drive()
            self.hybrid_trial_id += 1
            self.mode='lqr_balance' if balance_only else 'swing_balance'
            # Enable uses blocking serial transactions in this same executor.
            # Schedule a full first period instead of servicing an overdue timer.
            self.last_loop = time.monotonic()
            self.control_timer.reset()
        return self.response(response, action)

    def on_stop(self, request, response):
        def action():
            if self.disable() is False:
                raise ValueError('stop NOT confirmed; use physical E-stop')
        return self.response(response, action)

    def on_reset(self, request, response):
        def action():
            sample, error = self.snapshot()
            if self.transport_failed or error:
                raise ValueError('restart node after repairing serial/encoder failure')
            if not self.fresh() or sample[1][3] or sample[1][4]:
                raise ValueError('fresh sensors and clear endstops required')
            self.disable()
            self.fault = ''
            self.homed = False
        return self.response(response, action)

    def home_step(self, pos, rpm, left, now):
        """Bounded commissioning homing; reference is the LEFT RELEASE edge.

        Limits are fault thresholds, not guaranteed stopping distances.
        Requires a clear left switch at entry and a tested positive direction.
        """
        position = self.geo.position(pos)
        if self.mode == 'home_seek':
            with self.lock:
                hit = self.home_left_seen or left
            if hit:
                self.motor.velocity(0)
                self.output = 0.0
                self.home_reference = position
                self.home_last_position = position
                self.home_stationary = 0
                self.home_stop_start = now
                self.mode = 'home_stopping'
                return 0.0
            travel = self.x - self.home_reference
            if travel > 0.002 or -travel > self.p['track_length_m'] + 0.005:
                self.fail('homing seek direction/distance bound exceeded')
                return 0.0
            return -self.p['home_velocity_m_s']
        if self.mode == 'home_stopping':
            if abs(position - self.home_reference) > 0.003:
                self.fail('homing stop exceeded 3mm travel bound')
                return 0.0
            if now - self.home_stop_start > self.p['stop_timeout_s']:
                self.fail('homing standstill confirmation timeout')
                return 0.0
            still = (abs(rpm) <= self.p['stationary_rpm'] and
                     abs(position - self.home_last_position) <= 0.0001)
            self.home_last_position = position
            self.home_stationary = self.home_stationary + 1 if still else 0
            if self.home_stationary >= 3:
                # Do not infer the exact activation point from stopped position.
                self.home_reference = position
                self.mode = 'home_backoff'
            return 0.0
        if self.mode == 'home_backoff':
            travel = position - self.home_reference
            if travel < -0.002:
                self.fail('homing backoff moved in wrong direction')
                return 0.0
            with self.lock:
                if not self.home_left_released and not left:
                    self.home_left_released = True
                    self.origin = position
                    self.x = 0.0
                released = self.home_left_released
            if not released and travel >= 0.005:
                self.fail('left switch did not release within 5mm backoff')
                return 0.0
            if travel > self.p['soft_margin_m'] + self.p['stop_margin_m'] + 0.010:
                self.fail('homing backoff distance bound exceeded')
                return 0.0
            if released and self.x >= self.p['soft_margin_m'] + self.p['stop_margin_m']:
                self.disable()
                if self.stop_confirmed:
                    self.x = self.geo.position(self.motor.number('POS')) - self.origin
                    lo = self.p['soft_margin_m']
                    hi = self.p['track_length_m'] - lo
                    if not lo <= self.x <= hi:
                        self.fail('homing final position outside soft limits')
                    else:
                        self.homed = True
                return 0.0
            return self.p['home_velocity_m_s']
        self.fail('unknown homing state')
        return 0.0

    def tick(self):
        now = time.monotonic()
        dt = now - self.last_loop
        self.last_loop = now
        self.loop_dt_s = dt
        sample, error = self.snapshot()
        if error:
            if not self.fault:
                self.fail(error, transport=True)
            return
        if not self.fresh():
            if self.enabled:
                self.fail('sensor timeout')
            return
        if self.transport_failed:
            return
        # Sensor-thread latch preserves brief endstop events between 50Hz ticks.
        if self.enabled:
            with self.lock:
                event = getattr(self, 'limit_latched', '')
            if event:
                self.fail(event)
                return
            if sample[1][4] or (sample[1][3] and not self.mode.startswith('home_')):
                self.fail('endstop active before motor polling')
                return
        try:
            query_start = time.monotonic()
            pos = self.motor.number('POS') if self.motor else 0
            rpm = self.motor.number('GN') if self.motor else 0
            self.motor_roundtrip_s = time.monotonic() - query_start if self.motor else 0.0
            self.x = self.geo.position(pos) - self.origin
            self.v = self.geo.position(rpm * self.p['motor_counts_per_rev'] / 60)
            # Use the latest Arduino sample after blocking motor queries.
            sample, sensor_error = self.snapshot()
            if sensor_error or not self.fresh():
                self.fail('sensor unavailable after motor polling')
                return
            received, frame = sample
            seq, board_us, count, left, right, z, invalid = frame
            # x4 quadrature. Zero service records DOWN; theta=0 is UPRIGHT.
            self.theta = wrap(self.p['pole_sign'] * (((count - self.pole_zero + 2**31) % 2**32) - 2**31)
                              * 2 * math.pi / (4 * self.p['pole_ppr']) + math.pi)
            if self.last_pole is not None and seq != self.last_pole[0]:
                oldseq, oldus, oldcount = self.last_pole
                elapsed = ((board_us - oldus) & 0xffffffff) * 1e-6
                delta = ((count - oldcount + 2**31) % 2**32) - 2**31
                if elapsed > 0:
                    self.omega = self.p['pole_sign'] * delta * 2 * math.pi / (4 * self.p['pole_ppr'] * elapsed)
            self.last_pole = (seq, board_us, count)
            filtered_angle,filtered_rate=self.hybrid_observer.update(
                self.theta,self.v,dt,free_motion=self.enabled)
            self.hybrid_state=State(self.x,self.v,filtered_angle,filtered_rate)
            self.state_at = time.monotonic()
            if self.enabled:
                # Serial transactions may consume time; recheck the newest sensor packet.
                newest, sensor_error = self.snapshot()
                if sensor_error or not self.fresh():
                    self.fail('sensor freshness lost during motor transaction')
                    return
                left, right = newest[1][3:5]
                with self.lock:
                    event = self.limit_latched
                if event:
                    self.fail(event)
                    return
                if dt > self.p['sensor_timeout_s']:
                    self.fail('control loop overrun')
                    return
                if left and right or right:
                    self.fail('right endstop active (or both endstops active)')
                    return
                target = 0.0
                if self.mode.startswith('home_'):
                    if now - self.home_start > self.p['home_timeout_s']:
                        self.fail('homing timeout')
                        return
                    target = self.home_step(pos, rpm, left, time.monotonic())
                    if not self.enabled:
                        return
                else:
                    if left:
                        self.fail('left endstop active')
                        return
                    lo, hi = self.p['soft_margin_m'], self.p['track_length_m'] - self.p['soft_margin_m']
                    if not lo <= self.x <= hi:
                        self.fail('cart outside soft limits')
                        return
                    if self.mode in ('swing_balance','lqr_balance'):
                        if (time.monotonic()-received>.015 or self.motor_roundtrip_s>.015):
                            self.fail('hybrid: sensor age or motor roundtrip exceeds 15ms')
                            return
                        try:
                            target=self.hybrid.step(
                                self.hybrid_state,dt,self.output)
                        except ValueError as exc:
                            self.fail('hybrid: '+str(exc))
                            return
                        if self.hybrid.mode=='done':
                            self.disable()
                            return
                        self.mode='lqr_balance' if self.hybrid.mode=='balance' else 'swing_balance'
                    elif self.mode == 'balance':
                        if abs(self.theta) > self.p['balance_angle_limit_rad']:
                            self.fail('pole outside balance capture window')
                            return
                        state = (self.x - self.p['target_position_m'], self.v, self.theta, self.omega)
                        target = -sum(k * s for k, s in zip(self.p['balance_gains'], state))
                    elif now - self.command_time <= self.p['command_timeout_s']:
                        target = self.command
                    else:
                        # Stale command disables immediately; requires explicit re-enable.
                        if self.command_time != -math.inf:
                            self.fail('velocity command timeout')
                            return
                    target = guarded_velocity(target, self.x, left, right, lo, hi,
                                              self.p['max_velocity_m_s'],
                                              self.p['braking_deceleration_m_s2'],
                                              self.p['stop_margin_m'])
                if self.enabled:
                    # Rate limit, then reapply envelope so slowing isn't delayed by the ramp.
                    step = self.p['max_acceleration_m_s2'] * min(dt, 1 / self.p['loop_hz'] * 2)
                    self.output += max(-step, min(step, target - self.output))
                    if not self.mode.startswith('home_'):
                        self.output = guarded_velocity(self.output, self.x, left, right, lo, hi,
                                                       self.p['max_velocity_m_s'],
                                                       self.p['braking_deceleration_m_s2'],
                                                       self.p['stop_margin_m'])
                    if self.mode in ('swing_balance','lqr_balance'):
                        # Reapply INNER hybrid envelope after slew too, so ramp
                        # limits cannot delay braking at its 50–100mm boundary.
                        self.output=guarded_velocity(self.output,self.x,left,right,
                            self.hybrid.lower,self.hybrid.upper,self.hybrid.vmax,
                            self.hybrid.braking,self.hybrid.margin)
                    command_rpm = self.geo.rpm(self.output)
                    command_rpm = max(-self.p['max_motor_rpm'], min(self.p['max_motor_rpm'], command_rpm))
                    self.motor.velocity(command_rpm)
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = ['cart_joint', 'pole_joint']
            message.position = [self.x, self.theta]
            message.velocity = [self.v, self.omega]
            self.pub.publish(message)
            self.telemetry.publish(String(data=json.dumps(dict(
                host_monotonic_ns=time.monotonic_ns(), loop_dt_s=dt,
                motor_roundtrip_s=self.motor_roundtrip_s,
                sensor_age_s=time.monotonic()-received,
                motor_available=self.motor is not None, homed=self.homed,
                pole_calibrated=self.pole_calibrated,
                cart_m=self.x, cart_m_s=self.v, pole_rad=self.theta,
                pole_rad_s=self.omega, command_m_s=self.output,
                mode=self.mode, fault=self.fault,
                hybrid=self.hybrid.report() if self.hybrid else None,
                hybrid_trial_id=self.hybrid_trial_id,
                estimated_pole_rad=self.hybrid_state.theta,
                estimated_pole_rad_s=self.hybrid_state.omega))))
        except Exception as exc:
            self.fail('motor/protocol failure: ' + str(exc), transport=True)

    def publish_status(self):
        sample, error = self.snapshot()
        frame = sample[1] if sample else None
        self.status.publish(String(data=json.dumps(dict(
            mode=self.mode, enabled=self.enabled, homed=self.homed,
            motor_available=self.motor is not None, stop_confirmed=self.stop_confirmed,
            limit_latched=self.limit_latched,
            stop_trace=getattr(self.motor, 'stop_trace', []),
            sensor_packets=self.sensor_packets, bad_frames=self.sensor_bad_frames,
            dropped_frames=self.sensor_dropped,
            loop_dt_s=self.loop_dt_s, motor_roundtrip_s=self.motor_roundtrip_s,
            pole_calibrated=self.pole_calibrated, fault=self.fault,
            sensor_fresh=self.fresh(), sensor_error=error,
            left_blocked=frame[3] if frame else None,
            right_blocked=frame[4] if frame else None,
            encoder_count=frame[2] if frame else None,
            index_raw=frame[5] if frame else None,
            invalid_transitions=frame[6] if frame else None,
            cart_m=self.x, pole_rad=self.theta, command_m_s=self.output,
            hybrid_enabled=self.p['hybrid_enabled'],
            hybrid_candidate_id=self.p['hybrid_candidate_id'],
            hybrid_trial_id=self.hybrid_trial_id,
            hybrid=self.hybrid.report() if self.hybrid else None,
        ))))

    def close(self):
        self.running = False
        if self.motor:
            self.motor.close()
        if self.arduino:
            self.arduino.close()
        if hasattr(self, 'thread'):
            self.thread.join(timeout=0.2)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Cartpole()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()