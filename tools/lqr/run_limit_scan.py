"""Bounded experimental scan: 100 mm/s, 1 m/s2 commanded ramp.
Does not edit node parameters. Short pulses only; no hardware qualification.
"""
import json
import argparse
import math
import time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64,String
from std_srvs.srv import SetBool,Trigger

HARD_SPEED=.100
HARD_RAMP=1.0

def plan(vmax,amax):
    if not (0<vmax<=HARD_SPEED and 0<amax<=HARD_RAMP):
        raise ValueError('Ceiling exceeds experimental 100mm/s or 1m/s²')
    return [(sign*v, .20, amax) for v in (.080,.100) if v<=vmax
            for sign in (1,-1)]

def position_ok(x,initial):
    return math.isfinite(x) and .050 <= x <= .100 and abs(x-initial)<=.020

def pre_stop_peak(events, start_ns, zero_request_ns, direction):
    samples=[e['payload'] for e in events if e['kind']=='telemetry'
             and start_ns<=e['payload']['host_monotonic_ns']<zero_request_ns
             and e['payload'].get('mode')=='manual']
    speeds=[q['cart_m_s'] for q in samples]
    if not speeds or not all(math.isfinite(v) for v in speeds):
        raise RuntimeError('No valid pre-stop feedback for pulse scoring')
    return max(0.,max(direction*v for v in speeds)),max(abs(v) for v in speeds),len(speeds)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--max-speed-mm-s',type=float,default=100.)
    parser.add_argument('--max-ramp',type=float,default=1.0)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    pulses=plan(args.max_speed_mm_s/1000,args.max_ramp)
    if not args.execute and not args.check_only:
        print(json.dumps(dict(mode='preview_no_motion',plan=[dict(v_m_s=v,ramp_m_s2=a,duration_s=d) for v,d,a in pulses]),indent=2))
        return
    rclpy.init()
    node=rclpy.create_node('commission_limit_scan')
    latest={'status':None,'status_at':0.,'telemetry':None,'telemetry_at':0.}
    events=[]
    path=Path('/logs')/('crosscentre_100_'+str(time.time_ns())+'.json')
    path.write_text('{}')
    def record(kind,payload):
        events.append(dict(kind=kind,receive_ns=time.monotonic_ns(),payload=payload))
    def status(msg):
        value=json.loads(msg.data)
        latest.update(status=value,status_at=time.monotonic())
        record('status',value)
    def telemetry(msg):
        value=json.loads(msg.data)
        latest.update(telemetry=value,telemetry_at=time.monotonic())
        record('telemetry',value)
    sub1=node.create_subscription(String,'/cartpole/status',status,10)
    sub2=node.create_subscription(String,'/cartpole/telemetry',telemetry,qos_profile_sensor_data)
    pub=node.create_publisher(Float64,'/cartpole/cmd_velocity',1)
    enable=node.create_client(SetBool,'/cartpole/enable')
    stop=node.create_client(Trigger,'/cartpole/stop')
    params=node.create_client(GetParameters,'/cartpole/cartpole/get_parameters')
    attempted=False
    confirmed=True
    summary={'completed':False,'pulses':[]}

    def call(client,request):
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Service unavailable')
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=3)
        if not future.done():raise RuntimeError('Service response timeout')
        response=future.result()
        if response is None or not response.success:raise RuntimeError(str(response))
        return response

    def publish(v):
        before=time.monotonic_ns()
        pub.publish(Float64(data=v))
        record('publication',dict(requested_m_s=v,before_ns=before,after_ns=time.monotonic_ns()))

    def wait_ready(enabled):
        since=time.monotonic()
        until=since+3
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.005)
            s=latest['status'];q=latest['telemetry']
            if (s and q and latest['status_at']>since and latest['telemetry_at']>since
                and s.get('enabled')==enabled and not s.get('fault')
                and s.get('sensor_fresh') and not s.get('left_blocked')
                and not s.get('right_blocked') and s.get('homed')
                and (enabled or s.get('stop_confirmed'))):
                return
        raise RuntimeError('No fresh healthy '+('enabled' if enabled else 'disabled')+' status')

    try:
        if not params.wait_for_service(timeout_sec=5):raise RuntimeError('Parameter service unavailable')
        keys=['max_velocity_m_s','max_acceleration_m_s2','max_motor_rpm',
              'balance_enabled','track_length_m','soft_margin_m',
              'motor_revs_per_pulley_rev','pulley_circumference_m',
              'loop_hz','home_velocity_m_s','braking_deceleration_m_s2','stop_margin_m']
        future=params.call_async(GetParameters.Request(names=keys))
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done() or future.result() is None:raise RuntimeError('Parameter query failed')
        values=future.result().values
        if len(values)!=len(keys):raise RuntimeError('Incomplete parameters')
        p={k:(v.bool_value if v.type==1 else v.integer_value if v.type==2
               else v.double_value if v.type==3 else None) for k,v in zip(keys,values)}
        if any(v is None for v in p.values()):raise RuntimeError('Unexpected parameter types')
        record('parameters',p)
        if (abs(p['max_velocity_m_s']-.10)>1e-8 or abs(p['max_acceleration_m_s2']-1.0)>1e-8
            or not 1455<=p['max_motor_rpm']<=1500 or p['balance_enabled']
            or abs(p['track_length_m']-.147)>1e-6 or abs(p['soft_margin_m']-.0135)>1e-6
            or abs(p['motor_revs_per_pulley_rev']-9.7)>1e-6
            or abs(p['pulley_circumference_m']-.04)>1e-6
            or abs(p['braking_deceleration_m_s2']-.1)>1e-8
            or abs(p['stop_margin_m']-.005)>1e-8
            or abs(p['loop_hz']-50)>1e-6 or not 0 < p['home_velocity_m_s'] <= .01001):
            raise RuntimeError('Parameters do not match reviewed pulse-test setup: '+str(p))
        wait_ready(False)
        initial=latest['telemetry']['cart_m']
        if not .070<=initial<=.077:raise RuntimeError('Start near centre (70–77mm)')
        if not pub.get_subscription_count():raise RuntimeError('No velocity subscriber')
        summary['start_m']=initial
        print('START:',initial,'m',flush=True)
        if args.check_only:
            summary.update(completed=True,outcome='preflight_passed_no_motion')
            print('PREFLIGHT PASSED: no motion commands sent.',flush=True)
            return
        summary['ceilings']=dict(speed_m_s=args.max_speed_mm_s/1000,publisher_ramp_m_s2=args.max_ramp)
        for velocity,duration,acceleration in pulses:
            # Approach each pulse start slowly, so the fast pulse crosses centre.
            target=.062 if velocity>0 else .085
            attempted=True;confirmed=False
            call(enable,SetBool.Request(data=True))
            wait_ready(True)
            deadline=time.monotonic()+20
            while True:
                rclpy.spin_once(node,timeout_sec=.005)
                now=time.monotonic();q=latest['telemetry'];s=latest['status']
                if (now-latest['telemetry_at']>.08 or now-latest['status_at']>.5
                    or q.get('fault') or q.get('mode')!='manual'
                    or not s.get('enabled') or not s.get('sensor_fresh')
                    or s.get('left_blocked') or s.get('right_blocked')
                    or not .050<=q['cart_m']<=.100):
                    raise RuntimeError('Reposition interlock')
                error=target-q['cart_m']
                if abs(error)<.0004:break
                if now>deadline:raise RuntimeError('Reposition timeout')
                publish(max(-.005,min(.005,error*2)))
            publish(0.)
            print('POSITION STOP:',call(stop,Trigger.Request()),flush=True)
            confirmed=True
            wait_ready(False)
            initial=latest['telemetry']['cart_m']
            if abs(initial-target)>.001:raise RuntimeError('Pulse start outside tolerance')
            confirmed=False
            call(enable,SetBool.Request(data=True))
            wait_ready(True)
            print('PULSE:',velocity,'m/s for',duration,'s; publisher ramp',acceleration,'m/s2',flush=True)
            begin=time.monotonic();next_send=begin;last_send=begin;command=0.;peak=0.
            while time.monotonic()-begin<duration:
                rclpy.spin_once(node,timeout_sec=.002)
                now=time.monotonic();q=latest['telemetry'];s=latest['status']
                if (now-latest['telemetry_at']>.08 or now-latest['status_at']>.5
                    or not q or q.get('fault') or q.get('mode')!='manual'
                    or not s.get('enabled') or not s.get('sensor_fresh')
                    or s.get('left_blocked') or s.get('right_blocked')):
                    raise RuntimeError('Freshness/state interlock during pulse')
                if not position_ok(q['cart_m'],initial):raise RuntimeError('Position guard exceeded')
                if abs(q['cart_m_s'])>abs(velocity)*1.25+.002:
                    raise RuntimeError('Velocity overshoot guard exceeded')
                peak=max(peak,math.copysign(1,velocity)*q['cart_m_s'])
                if now>=next_send:
                    step=acceleration*max(0.,now-last_send)
                    command+=max(-step,min(step,velocity-command))
                    publish(command)
                    last_send=now;next_send=now+.02
            publish(0.)
            print('STOP:',call(stop,Trigger.Request()),flush=True)
            confirmed=True
            wait_ready(False)
            q=latest['telemetry']
            if not position_ok(q['cart_m'],initial):raise RuntimeError('Post-stop position guard exceeded')
            trace=latest['status'].get('stop_trace', [])
            if not trace or trace[0].get('kind')!='zero_request':
                raise RuntimeError('No timestamped stop trace for pulse scoring')
            peak,maximum_speed,scored_samples=pre_stop_peak(events,int(begin*1e9),
                trace[0]['host_monotonic_ns'],math.copysign(1,velocity))
            if maximum_speed>abs(velocity)*1.25+.002:
                raise RuntimeError('Recorded pre-stop overspeed guard exceeded')
            result=dict(requested_m_s=velocity,pulse_s=duration,publisher_ramp_m_s2=acceleration,
                        scored_pre_stop_samples=scored_samples,tracking_criterion_met=peak>=abs(velocity)*.90,
                        observed_peak_in_requested_direction_m_s=peak,stop_position_m=q['cart_m'])
            summary['pulses'].append(result)
            print('RESULT:',json.dumps(result),flush=True)
            if peak<abs(velocity)*.90:
                raise RuntimeError('Less than 90% of requested speed observed; scan halted')
            # Stationary interval, keep receiving fresh data; drive is disabled.
            until=time.monotonic()+1
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        summary['completed']=True
        summary['outcome']='completed_within_test_ceiling_not_hardware_maximum'
    except KeyboardInterrupt:
        summary['error']='Interrupted'
    except Exception as exc:
        summary['error']=str(exc)
    finally:
        if attempted and not confirmed:
            try:
                publish(0.)
                print('CLEANUP STOP:',call(stop,Trigger.Request()),flush=True)
                confirmed=True
            except Exception as exc:
                summary['stop_error']=str(exc)
                print('STOP UNCONFIRMED: use physical cutoff immediately.',flush=True)
        until=time.monotonic()+.5
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        passed=[r for r in summary['pulses'] if r.get('tracking_criterion_met')]
        summary['largest_requested_speed_passing_short_pulse_m_s']=max([abs(r['requested_m_s']) for r in passed],default=0.)
        summary['largest_requested_ramp_passing_short_pulse_m_s2']=max([r['publisher_ramp_m_s2'] for r in passed],default=0.)
        summary['motion_attempted']=attempted
        summary['stop_service_success']=confirmed
        summary['final_status']=latest['status']
        path.write_text(json.dumps(dict(summary=summary,events=events,
            limitation='Host timestamps and asynchronously polled feedback; no guaranteed stop distance. Publisher slew is a requested ramp, not measured physical acceleration. A successful scan establishes only the tested ceiling, not hardware maximum. No thermal/current/gearbox qualification. Not a balancing test.'),indent=2))
        print('RECORDING:',path,flush=True)
        print('SUMMARY:',json.dumps(summary,indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()
        if args.check_only:
            print('PREFLIGHT PASS' if summary.get('completed') and confirmed else 'PREFLIGHT FAIL', flush=True)
    if not summary['completed'] or not confirmed:raise SystemExit(1)

if __name__=='__main__':main()
