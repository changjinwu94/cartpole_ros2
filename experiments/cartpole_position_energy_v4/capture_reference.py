"""Run inside NORMAL running cartpole container, AFTER preparation.
Stops via service, then exports a fresh absolute-position/cart-position pair.
"""
import json
import time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters

def main():
    rclpy.init();n=rclpy.create_node('capture_position_reference');state={};received=0
    def update(msg):
        nonlocal state,received
        state=json.loads(msg.data);received=time.monotonic_ns()
    sub=n.create_subscription(String,'/cartpole/status',update,10)
    try:
        deadline=time.monotonic()+5
        while not state and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
        if not state or state.get('enabled') or not state.get('homed') or not state.get('pole_calibrated'):
            raise RuntimeError('Prepare home/calibration/centre first; start disabled')
        c=n.create_client(GetParameters,'/cartpole/cartpole/get_parameters')
        if not c.wait_for_service(timeout_sec=3):raise RuntimeError('No parameter service')
        expected={'motor_counts_per_rev':3000,'motor_revs_per_pulley_rev':9.7,
                  'pulley_circumference_m':.04,'motor_sign':1,'pole_ppr':500,'pole_sign':1,
                  'track_length_m':.147,'balance_enabled':False}
        f=c.call_async(GetParameters.Request(names=list(expected)))
        rclpy.spin_until_future_complete(n,f,timeout_sec=3)
        if not f.done() or f.result() is None:raise RuntimeError('Parameter query timeout')
        vals=f.result().values
        actual={k:(v.bool_value if v.type==1 else v.integer_value if v.type==2 else v.double_value if v.type==3 else None)
                for k,v in zip(expected,vals)}
        if len(actual)!=len(expected) or any(actual.get(k) is None or abs(actual[k]-v)>1e-8 for k,v in expected.items()):
            raise RuntimeError('Unexpected geometry: '+str(actual))
        stop=n.create_client(Trigger,'/cartpole/stop')
        if not stop.wait_for_service(timeout_sec=3):raise RuntimeError('No stop service')
        start=time.monotonic_ns();f=stop.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n,f,timeout_sec=3)
        if not f.done() or f.result() is None or not f.result().success:raise RuntimeError('Stop failed')
        floor=time.monotonic_ns();deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.02)
            trace=state.get('stop_trace',[])
            if received>floor and trace and trace[0].get('host_monotonic_ns',0)>=start:break
        else:raise RuntimeError('No fresh stop trace')
        if (state.get('enabled') or not state.get('homed') or not state.get('stop_confirmed')
            or state.get('fault') or not state.get('sensor_fresh') or state.get('sensor_error')
            or state.get('left_blocked') or state.get('right_blocked')
            or not .070<=state['cart_m']<=.077):raise RuntimeError('Unhealthy/not centred: '+str(state))
        feedback=[x for x in trace if x['kind']=='feedback']
        if not feedback or abs(feedback[-1]['motor_rpm'])>5:raise RuntimeError('No stationary position reference')
        r=dict(captured_ns=time.monotonic_ns(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
               position_counts=feedback[-1]['position_counts'],cart_m=state['cart_m'],
               counts_per_rev=3000,gear_ratio=9.7,pulley_m=.04,source_status=state)
        path=Path('/logs/position_reference.json');path.write_text(json.dumps(r,indent=2))
        print('REFERENCE PASS:',path,'position_counts=',r['position_counts'],'cart_m=',r['cart_m'],flush=True)
    finally:n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
