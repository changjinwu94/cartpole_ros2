"""Exclusive-serial reference-profile experiment. NORMAL ROS NODE MUST BE STOPPED."""
import argparse
import json
import math
from pathlib import Path
import signal
import threading
import time
from gc_window import GcWindow
from state_push import state_push_target
from phase_core import DownCrossings, PhasePolicy
from energy_core import estimate, tapered_target, FINAL_ENERGY_THRESHOLD, FINAL_MOVE_CAP, TRANSITION_START
from profile_core import EXPECTED, SCALE, PERIOD, targets, cart_position, validate_reference, position_guard, pole_angle, parse_frame

FINAL_PUSH_ENERGY = 0.83
FINAL_PUSH_FRACTION = 0.40


class PreflightComplete(Exception):
    pass

class Sensors:
    def __init__(self,link,events):
        self.link=link;self.events=events;self.lock=threading.RLock();self.done=threading.Event()
        self.latest=None;self.error='';self.history=[];self.down=None;self.upright=False
        self.phase=None;self.crossings=[];self.entry=None
        self.closest=math.pi;self.active=False;self.max_gap=0.;self.previous_at=None
        self.thread=threading.Thread(target=self.read,daemon=True);self.thread.start()
    def read(self):
        previous=None
        while not self.done.is_set():
            raw=self.link.read_until(b'\n',160)
            if not raw:continue
            try:q=parse_frame(raw)
            except (ValueError,UnicodeError):
                with self.lock:
                    if self.active:self.error='Malformed/invalid sensor frame'
                continue
            now=time.monotonic_ns()
            with self.lock:
                if previous is not None and ((q['sequence']-previous)&0xffffffff)!=1:
                    if self.active:self.error='Sensor sequence gap/reset'
                previous=q['sequence']
                if self.active and self.previous_at is not None:
                    gap=(now-self.previous_at)/1e9;self.max_gap=max(self.max_gap,gap)
                    if gap>.05:self.error='Latched raw serial gap >50ms'
                self.previous_at=now
                if self.active and (q['left'] or q['right']):self.error='Latched endstop event'
                q['at_ns']=now;self.latest=q;self.history.append(q)
                self.history=self.history[-260:]
                if self.active and self.down is not None:
                    theta=pole_angle(q['count'],self.down);self.closest=min(self.closest,abs(theta))
                    crossing=self.phase.update(q['count'],now)
                    if crossing:
                        self.crossings.append(crossing)
                        self.events.append(dict(kind='down_crossing',**crossing))
                    if abs(theta)<=math.radians(12):
                        if not self.upright:self.entry=estimate(self.history,self.down)
                        self.upright=True
                self.events.append(dict(kind='sensor_raw',host_monotonic_ns=now,payload=q))
    def snapshot(self):
        with self.lock:
            if self.error:raise RuntimeError(self.error)
            q=dict(self.latest) if self.latest else None
            if not q or time.monotonic_ns()-q['at_ns']>50_000_000:raise RuntimeError('Sensor freshness >50ms')
            if q['left'] or q['right']:raise RuntimeError('Endstop active')
            return q,self.upright
    def settle(self):
        deadline=time.monotonic()+6
        while time.monotonic()<deadline:
            time.sleep(.01)
            with self.lock:
                now=time.monotonic_ns();h=[q for q in self.history if now-q['at_ns']<=1_100_000_000]
                if h and now-h[0]['at_ns']>=1_000_000_000 and max(q['count'] for q in h)-min(q['count'] for q in h)<=2:
                    self.down=h[-1]['count'];self.phase=DownCrossings(self.down);self.active=True;self.previous_at=now
                    self.snapshot();return
        raise RuntimeError('Pole must hang freely down and settle before test')
    def entry_energy(self):
        with self.lock:return dict(self.entry) if self.entry is not None else estimate(self.history,self.down)
    def energy(self):
        with self.lock:return estimate(self.history,self.down)
    def take_crossings(self):
        with self.lock:
            result=self.crossings[:];self.crossings.clear();return result
    def close(self):
        self.done.set();self.thread.join(timeout=1)
        if self.thread.is_alive():raise RuntimeError('Sensor thread did not stop')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    mode=ap.add_mutually_exclusive_group();mode.add_argument('--check-only',action='store_true');mode.add_argument('--execute',action='store_true')
    ap.add_argument('--duration',type=float,default=10.,help='Seconds, maximum20')
    ap.add_argument('--pole-down',action='store_true')
    ap.add_argument('--policy', choices=('energy-v4', 'state-push', 'final-push'), default='energy-v4')
    args=ap.parse_args()
    if not math.isfinite(args.duration) or not 0<args.duration<=20:ap.error('duration must be >0 and <=20s')
    if not args.check_only and not args.execute:
        print(json.dumps(dict(mode='preview_no_serial_access',offset_counts=[11000,-9000],trigger='down_crossing',minimum_interval_s=.15,
            duration_s=args.duration,policy=args.policy,profile_required=EXPECTED)));return
    if not args.pole_down:ap.error('--pole-down required: physically leave pole hanging down')
    import serial
    from motor_io import Motor
    events=[];summary=dict(completed=False,motion_attempted=False,stop_confirmed=None,control_mode='phase_locked_energy_taper_v4' if args.policy=='energy-v4' else args.policy.replace('-', '_') + '_swing_only',transition_start=TRANSITION_START,final_energy_threshold=FINAL_ENERGY_THRESHOLD,final_move_cap=FINAL_MOVE_CAP,crossing_hysteresis_deg=5.,minimum_command_interval_s=.15)
    path=Path('/logs')/('position_energy_v4_test_'+str(time.time_ns())+'.json');path.write_text('{}')
    m=None;link=None;sensors=None;gcw=GcWindow();armed=False
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    try:
        ref=json.loads(Path('/logs/position_reference.json').read_text())
        validate_reference(ref,time.monotonic_ns(),Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        m=Motor('/dev/cartpole_motor',115200,.03) # exclusive, DI; normal node must be stopped
        profile={k:m.number(k) for k in EXPECTED};summary['drive_profile']=profile
        if profile!=EXPECTED:raise RuntimeError('Stored drive profile differs from screenshot; nothing enabled: '+str(profile))
        pos=m.number('POS');rpm=m.number('GN')
        if abs(rpm)>5 or abs(pos-ref['position_counts'])>20:raise RuntimeError('Cart/reference moved; recapture while centred')
        start=pos;endpoints=targets(start)
        if not all(position_guard(p,ref) for p in endpoints):raise RuntimeError('Target outside trial bounds')
        summary.update(reference=ref,start_counts=start,targets_counts=endpoints,
                       targets_cart_m=[cart_position(p,ref) for p in endpoints],duration_requested_s=args.duration)
        # Confirmed disabled by Motor initializer; no motion before explicit execute.
        gcw.begin()
        link=serial.Serial('/dev/cartpole_arduino',115200,timeout=.01,write_timeout=.03,exclusive=True)
        sensors=Sensors(link,events);sensors.settle()
        if args.check_only:
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:sensors.snapshot();time.sleep(.005)
            summary.update(completed=True,outcome='preflight_passed_no_enable');raise PreflightComplete()
        # Use drive position loop, not ROS velocity slew. No AC/DEC/SP/current/EEPROM writes.
        m.command('CONTMOD');m.command('SOR0');m.command('V0')
        # Load the CURRENT position before enabling: stale previous targets cannot launch motion.
        m.command('LA'+str(start))
        sensors.snapshot();summary['motion_attempted']=True;armed=True
        m.command('EN')
        begin=time.monotonic();last_poll=begin;index=0;policy=PhasePolicy(time.monotonic_ns())
        final_push_sent=False
        while time.monotonic()-begin<args.duration:
            now=time.monotonic();q,upright=sensors.snapshot()
            if upright:
                summary['upright_entered']=True;summary['entry_estimate']=sensors.entry_energy();break
            if now-last_poll>.05:raise RuntimeError('Motor feedback/control interval >50ms')
            before=time.monotonic_ns();pos=m.number('POS');rpm=m.number('GN');after=time.monotonic_ns()
            events.append(dict(kind='motor_feedback',before_ns=before,after_ns=after,position_counts=pos,motor_rpm=rpm,cart_m=cart_position(pos,ref)))
            if (after-before)>50_000_000:raise RuntimeError('Motor query roundtrip >50ms')
            if not position_guard(pos,ref):raise RuntimeError('Cart position guard exceeded')
            if abs(rpm)>4200:raise RuntimeError('Motor speed >4200rpm')
            sensors.snapshot();last_poll=time.monotonic()
            if sensors.upright:
                summary['upright_entered']=True;summary['entry_estimate']=sensors.entry_energy();break
            if last_poll-begin>=args.duration:break
            decision_ns=time.monotonic_ns()
            direction,reason=policy.choose(decision_ns,sensors.take_crossings())
            if direction is not None:
                if index>=40:raise RuntimeError('Command count limit40 reached')
                endpoint=endpoints[0 if direction>0 else 1]
                energy=sensors.energy()
                if final_push_sent:
                    summary['outcome']='final_push_returned_without_upright_entry'
                    break
                if args.policy == 'final-push' and energy['energy_fraction'] >= FINAL_PUSH_ENERGY:
                    if direction*(endpoint-pos)<=0:
                        raise RuntimeError('Final push endpoint is behind the commanded direction')
                    fraction=FINAL_PUSH_FRACTION
                    target=pos+round(fraction*(endpoint-pos))
                    final_push_sent=True
                    reason='reduced_final_push'
                elif args.policy in ('state-push', 'final-push'):
                    target,fraction=state_push_target(pos,endpoint,energy['pole_rate_rad_s'],direction)
                else:
                    target,fraction=tapered_target(pos,endpoint,energy['energy_fraction'],direction)
                events.append(dict(kind='energy_decision',host_monotonic_ns=time.monotonic_ns(),
                    current_counts=pos,endpoint_counts=endpoint,target_counts=target,
                    move_fraction=fraction,direction=direction,reason=reason,**energy))
                if target is None:
                    time.sleep(.002)
                    continue
                if not position_guard(target,ref):raise RuntimeError('Tapered target outside travel guard')
                before=time.monotonic_ns();m.command('LA'+str(target));sensors.snapshot()
                if sensors.upright:
                    summary['upright_entered']=True;summary['entry_estimate']=sensors.entry_energy();break
                m.command('M');after=time.monotonic_ns()
                events.append(dict(kind='position_target',target_counts=target,before_ns=before,after_ns=after,
                                   decision_ns=decision_ns,reason=reason,direction=direction))
                index+=1
            time.sleep(.002)
        summary.update(completed=True,commands_sent=index,duration_s=time.monotonic()-begin,
                       outcome='upright_entry' if summary.get('upright_entered') else summary.get('outcome','bounded_time_limit'))
    except PreflightComplete:pass
    except (Exception,KeyboardInterrupt) as exc:summary['error']=str(exc) or 'Interrupted'
    finally:
        if m is not None and armed:
            try:
                summary['stop_confirmed']=m.stop(timeout=1.,stationary_rpm=5.)
                events.append(dict(kind='stop_trace',trace=m.stop_trace))
                if not summary['stop_confirmed']:
                    summary['completed']=False;print('STOP UNCONFIRMED: use physical cutoff immediately.',flush=True)
            except Exception as exc:
                summary.update(completed=False,stop_confirmed=False,stop_error=str(exc))
                print('STOP UNCONFIRMED: use physical cutoff immediately.',flush=True)
        decisions=[e for e in events if e['kind']=='energy_decision']
        summary['tapered_moves']=sum(e['target_counts'] is not None and e['move_fraction']<1. for e in decisions)
        summary['held_crossings']=sum(e['target_counts'] is None for e in decisions)
        if sensors:
            summary.update(closest_upright_deg=math.degrees(sensors.closest),raw_max_gap_ms=sensors.max_gap*1000)
            try:sensors.close()
            except Exception as exc:summary.update(completed=False,sensor_shutdown_error=str(exc))
        if link:link.close()
        if m:m.io.close() # do not call close(), which issues a second stop
        gcw.close();summary['gc_management']=gcw.report()
        path.write_text(json.dumps(dict(summary=summary,events=events,limitation='Exclusive serial test bypasses ROS motion gates. Software guards do not guarantee physical stopping distance. Pole is not balanced/caught. Reference mapping uses latest ROS stop feedback and must be recaptured for each invocation.'),indent=2))
        print('RECORDING:',path,flush=True);print('SUMMARY:',json.dumps(summary,indent=2),flush=True)
        print(('PREFLIGHT ' if args.check_only else 'TEST ')+('PASS' if summary['completed'] else 'FAIL'),flush=True)
    if not summary['completed']:raise SystemExit(1)

if __name__=='__main__':main()
