"""Latch down-crossings after excursions, independent of serial/ROS."""
import math

class DownCrossings:
    def __init__(self,down,hysteresis_deg=5.):
        self.down=down;self.band=math.radians(hysteresis_deg)
        self.side=0;self.previous=None;self.previous_ns=None
    def update(self,count,stamp_ns):
        if self.previous_ns is not None and stamp_ns<=self.previous_ns:
            raise ValueError('Non-increasing phase timestamp')
        phi=(count-self.down)*2*math.pi/2000
        phi=math.atan2(math.sin(phi),math.cos(phi))
        result=None
        if self.previous is not None and abs(phi-self.previous)<math.pi:
            if self.side==-1 and self.previous<0<=phi:
                result=dict(direction=1,at_ns=stamp_ns,phi_rad=phi);self.side=0
            elif self.side==1 and self.previous>0>=phi:
                result=dict(direction=-1,at_ns=stamp_ns,phi_rad=phi);self.side=0
        if phi<=-self.band:self.side=-1
        elif phi>=self.band:self.side=1
        self.previous=phi;self.previous_ns=stamp_ns
        return result

class PhasePolicy:
    def __init__(self,start_ns):
        self.start=start_ns;self.last_command=None;self.last_target=None;self.last_cross=start_ns
    def begin_stroke(self, now_ns, direction, reason):
        self.last_command = now_ns
        self.last_target = direction
        return direction, reason

    def choose(self,now_ns,crossings):
        if self.last_command is None:
            return self.begin_stroke(now_ns, 1, 'initial_bounded_nudge')
        direction=None
        for crossing in crossings:
            if now_ns-crossing['at_ns']>30_000_000:raise RuntimeError('Down-crossing delivery >30ms')
            if crossing['at_ns']<self.start:continue
            self.last_cross=crossing['at_ns']
            if crossing['direction']==self.last_target:continue
            if now_ns-self.last_command<150_000_000:
                raise RuntimeError('Down-crossing command interval <150ms')
            direction=crossing['direction']
        if now_ns-self.last_cross>2_000_000_000:raise RuntimeError('No down-crossing for2s; pole stalled')
        if direction is None:return None,'waiting_for_down_crossing'
        return self.begin_stroke(now_ns, direction, 'down_crossing')
