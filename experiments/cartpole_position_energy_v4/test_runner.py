"""Hardware-free orchestration tests using fake serial endpoints and clock."""
import contextlib
import io
import json
import math
import types
import unittest
from unittest.mock import patch
import run_position as runner
from profile_core import EXPECTED

class Tests(unittest.TestCase):
    def simulate(self,check=False,upright=False,bad_profile=False,stop_ok=True):
        clock=[1.];commands=[];writes=[];motor=[]
        ref=dict(boot_id='boot',captured_ns=1000000000,cart_m=.0735,position_counts=-2000,counts_per_rev=3000,gear_ratio=9.7,pulley_m=.04)
        class Path:
            def __init__(self,name):self.name=str(name)
            def __truediv__(self,name):return Path(self.name+'/'+name)
            def read_text(self):return 'boot' if self.name.endswith('boot_id') else json.dumps(ref)
            def write_text(self,text):writes.append(text)
            def __str__(self):return self.name
        class Motor:
            def __init__(self,*a):self.pos=-2000;self.target=-2000;self.io=types.SimpleNamespace(close=lambda:None);self.stop_trace=[];motor.append(self)
            def number(self,c):
                if c=='POS':return self.pos
                if c=='GN':return 0
                return EXPECTED[c]+(1 if bad_profile and c=='GAC' else 0)
            def command(self,c):
                commands.append(c)
                if c.startswith('LA'):self.target=int(c[2:])
                if c=='M':self.pos=self.target
            def stop(self,**kw):commands.append('STOP');return stop_ok
        class Sensors:
            def __init__(self,*a):self.closest=math.pi;self.max_gap=0.;self.upright=False;self.epoch=clock[0];self.tick=-1
            def settle(self):pass
            def snapshot(self):
                self.upright=upright and 'M' in commands
                return {},self.upright
            def energy(self):return dict(energy_fraction=0.,pole_rate_rad_s=0.,source_ns=int(clock[0]*1e9))
            def entry_energy(self):return self.energy()
            def take_crossings(self):
                tick=int((clock[0]-self.epoch)/.25)
                if tick==self.tick:return []
                self.tick=tick
                return [dict(direction=1 if tick%2==0 else -1,at_ns=int(clock[0]*1e9))]
            def close(self):pass
        class GC:
            def begin(self):pass
            def energy(self):return dict(energy_fraction=0.,pole_rate_rad_s=0.,source_ns=int(clock[0]*1e9))
            def entry_energy(self):return self.energy()
            def take_crossings(self):
                tick=int((clock[0]-self.epoch)/.25)
                if tick==self.tick:return []
                self.tick=tick
                return [dict(direction=1 if tick%2==0 else -1,at_ns=int(clock[0]*1e9))]
            def close(self):pass
            def report(self):return dict(restored=True)
        def sleep(dt):clock[0]+=dt
        fake_time=types.SimpleNamespace(monotonic=lambda:clock[0],monotonic_ns=lambda:int(clock[0]*1e9),time_ns=lambda:1,sleep=sleep)
        serial=types.SimpleNamespace(Serial=lambda *a,**kw:types.SimpleNamespace(close=lambda:None))
        args=['run_position.py','--pole-down','--duration','2','--check-only' if check else '--execute']
        code=0
        with patch.dict('sys.modules',{'serial':serial,'motor_io':types.SimpleNamespace(Motor=Motor)}),patch('sys.argv',args),patch.object(runner,'Path',Path),patch.object(runner,'Sensors',Sensors),patch.object(runner,'GcWindow',GC),patch.object(runner,'time',fake_time),patch.object(runner.signal,'signal'),contextlib.redirect_stdout(io.StringIO()):
            try:runner.main()
            except SystemExit as e:code=e.code
        return commands,json.loads(writes[-1])['summary'],code
    def test_check_only_never_enables(self):
        c,s,code=self.simulate(check=True);self.assertNotIn('EN',c);self.assertNotIn('M',c);self.assertEqual(code,0)
    def test_current_target_loaded_before_enable(self):
        c,_,_=self.simulate();self.assertLess(c.index('LA-2000'),c.index('EN'))
    def test_phase_commands_are_bounded(self):
        c,s,code=self.simulate();self.assertGreaterEqual(c.count('M'),2);self.assertLessEqual(c.count('M'),40);self.assertAlmostEqual(s['duration_s'],2.,delta=.01);self.assertTrue(s['stop_confirmed']);self.assertEqual(code,0)
    def test_profile_mismatch_never_enables(self):
        c,s,code=self.simulate(bad_profile=True);self.assertNotIn('EN',c);self.assertEqual(code,1)
    def test_upright_stops_early(self):
        c,s,_=self.simulate(upright=True);self.assertEqual(c.count('M'),1);self.assertEqual(s['outcome'],'upright_entry');self.assertIn('STOP',c)
    def test_failed_stop_returns_failure(self):
        _,s,code=self.simulate(stop_ok=False);self.assertFalse(s['completed']);self.assertEqual(code,1)
if __name__=='__main__':unittest.main()
