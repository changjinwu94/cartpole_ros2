"""Home, calibrate a physically hanging-down pole, centre, then preflight.
Operator must place pole down before invocation. Does not identify gravity pose.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger


def centre_command(x, home_speed):
    if not math.isfinite(home_speed) or not 0<home_speed<=.01:
        raise RuntimeError("Home/centring speed must be >0 and <=0.01 m/s")
    if not math.isfinite(x) or not .018<=x<=.129:
        raise RuntimeError('Centre travel guard exceeded')
    error=.0735-x
    return 0. if abs(error)<=.0003 else max(-home_speed,min(home_speed,2*error))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pole-down',action='store_true',required=True,
                        help='Operator confirms pole is hanging freely down')
    parser.parse_args()
    rclpy.init();node=rclpy.create_node('commission_prepare_cart')
    latest={};arrival={};generation={'status':0,'telemetry':0}
    def receive(kind,msg):
        latest[kind]=json.loads(msg.data);arrival[kind]=time.monotonic();generation[kind]+=1
    subscriptions=[node.create_subscription(String,'/cartpole/status',lambda m:receive('status',m),10),
        node.create_subscription(String,'/cartpole/telemetry',lambda m:receive('telemetry',m),qos_profile_sensor_data)]
    pub=node.create_publisher(Float64,'/cartpole/cmd_velocity',1)
    clients={name:node.create_client(SetBool if name=='enable' else Trigger,'/cartpole/'+name)
             for name in ('home','zero_pole_down','enable','stop')}
    def call(name):
        c=clients[name]
        if not c.wait_for_service(timeout_sec=5):raise RuntimeError(name+' unavailable')
        request=SetBool.Request(data=True) if name=='enable' else Trigger.Request()
        f=c.call_async(request);rclpy.spin_until_future_complete(node,f,timeout_sec=5)
        if not f.done() or f.result() is None or not f.result().success:
            raise RuntimeError(name+' failed: '+str(f.result() if f.done() else 'timeout'))
        print(name.upper()+': OK',flush=True)
    def healthy(allow_left=False):
        now=time.monotonic();s=latest.get('status',{});q=latest.get('telemetry',{})
        return (now-arrival.get('status',-math.inf)<.5
                and now-arrival.get('telemetry',-math.inf)<.1
                and s.get('sensor_fresh') and s.get('motor_available')
                and not s.get('fault') and not s.get('sensor_error')
                and not s.get('right_blocked') and (allow_left or not s.get('left_blocked')))
    def wait(predicate,seconds,allow_left=False):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if latest and latest.get('status',{}).get('fault'):
                raise RuntimeError('Node fault: '+latest['status']['fault'])
            if healthy(allow_left) and predicate():return
        raise RuntimeError('Timed out waiting for healthy phase completion')
    attempted=False;ok=False
    try:
        wait(lambda:not latest['status'].get('enabled') and pub.get_subscription_count()>0,10)
        # Check geometry and motion envelope before starting homing.
        c=node.create_client(GetParameters,'/cartpole/cartpole/get_parameters')
        if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Parameter service unavailable')
        expected={'track_length_m':.147,'soft_margin_m':.0135,'stop_margin_m':.005,
                  'max_velocity_m_s':.1,'max_acceleration_m_s2':1.,'max_motor_rpm':1500.,
                  'braking_deceleration_m_s2':.1,'loop_hz':50.,'home_velocity_m_s':.01,
                  'balance_enabled':False}
        f=c.call_async(GetParameters.Request(names=list(expected)))
        rclpy.spin_until_future_complete(node,f,timeout_sec=5)
        if not f.done() or f.result() is None:raise RuntimeError('Parameter query failed')
        values=f.result().values
        actual={k:(v.bool_value if v.type==1 else v.integer_value if v.type==2 else v.double_value if v.type==3 else None)
                for k,v in zip(expected,values)}
        if (len(actual)!=len(expected) or any(actual.get(k) is None or
                (k!='home_velocity_m_s' and abs(actual[k]-v)>1e-8) for k,v in expected.items())
                or not 0<actual.get('home_velocity_m_s',0)<=.01):
            raise RuntimeError('Unexpected preparation parameters: '+str(actual))
        attempted=True
        call('home')
        after=time.monotonic()
        wait(lambda:arrival['status']>after and latest['status'].get('homed')
             and not latest['status'].get('enabled') and latest['status'].get('stop_confirmed'),130,True)
        print('HOME COMPLETED',flush=True)
        # Require stationary encoder for 1 second; pose is operator-confirmed.
        deadline=time.monotonic()+10;counts=[]
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if not healthy():raise RuntimeError('Unhealthy before calibration')
            now=time.monotonic();counts.append((now,latest['status']['encoder_count']))
            counts=[p for p in counts if now-p[0]<=1.1]
            if now-counts[0][0]>=1 and max(p[1] for p in counts)-min(p[1] for p in counts)<=2:break
        else:raise RuntimeError('Pole did not settle; leave it hanging down')
        call('zero_pole_down')
        after=time.monotonic()
        wait(lambda:arrival['status']>after and latest['status'].get('pole_calibrated'),3)
        home_speed=actual['home_velocity_m_s']
        print('CENTRING speed cap:',home_speed,'m/s (matches homing)',flush=True)
        call('enable');after=time.monotonic()
        wait(lambda:arrival['status']>after and latest['status'].get('enabled') and latest['status'].get('mode')=='manual',3)
        deadline=time.monotonic()+60;next_send=0
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.005)
            if not healthy() or not latest['status'].get('enabled') or latest['telemetry'].get('mode')!='manual':
                raise RuntimeError('Centre interlock')
            command=centre_command(latest['telemetry']['cart_m'],home_speed)
            if command==0.:break
            if time.monotonic()>=next_send:
                pub.publish(Float64(data=command));next_send=time.monotonic()+.02
        else:raise RuntimeError('Centre timeout')
        pub.publish(Float64(data=0.));call('stop');after=time.monotonic()
        wait(lambda:arrival['status']>after and not latest['status'].get('enabled')
             and latest['status'].get('stop_confirmed') and latest['status'].get('homed')
             and abs(latest['telemetry']['cart_m']-.0735)<.001,3)
        ok=True
        print('CENTRE COMPLETED:',latest['telemetry']['cart_m'],'m',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        print('PREPARATION ERROR:',exc,flush=True)
    finally:
        if attempted and not ok:
            try:pub.publish(Float64(data=0.));call('stop')
            except Exception as exc:print('STOP UNCONFIRMED: use physical cutoff.',exc,flush=True)
        node.destroy_node();rclpy.shutdown()
    if not ok:print('PREPARATION FAIL',flush=True);raise SystemExit(1)
    result=subprocess.run([sys.executable,str(Path(__file__).with_name('run_limit_scan.py')),'--check-only'])
    print('PREPARATION PASS' if result.returncode==0 else 'PREPARATION FAIL',flush=True)
    raise SystemExit(result.returncode)

if __name__=='__main__':main()
