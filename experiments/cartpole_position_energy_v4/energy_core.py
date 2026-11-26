"""Causal short-window rate and bounded energy-dependent position steps."""
import math
from profile_core import SCALE
INERTIA=.00707*.105**2/3+.00064*.100**2
MG=(.00707*.105/2+.00064*.100)*9.81
FULL_UNTIL=.65
MIN_COUNTS=100
TRANSITION_START=.75
FINAL_ENERGY_THRESHOLD=.85
FINAL_MOVE_CAP=.32

def estimate(samples,down):
    if len(samples)<3:raise RuntimeError('Not enough pole samples for energy estimate')
    latest=samples[-1]['at_ns']
    window=[q for q in samples if latest-q['at_ns']<=25_000_000]
    if len(window)<3 or window[-1]['at_ns']-window[0]['at_ns']<10_000_000:
        raise RuntimeError('Insufficient recent pole-rate window')
    ts=[(q['at_ns']-latest)/1e9 for q in window]
    angles=[(q['count']-down)*2*math.pi/2000 for q in window]
    mt=sum(ts)/len(ts);ma=sum(angles)/len(angles)
    denom=sum((t-mt)**2 for t in ts)
    if denom<=0:raise RuntimeError('Invalid pole-rate timestamps')
    omega=sum((t-mt)*(a-ma) for t,a in zip(ts,angles))/denom
    phi=angles[-1]
    fraction=(.5*INERTIA*omega**2+MG*(1-math.cos(phi)))/(2*MG)
    if not math.isfinite(fraction):raise RuntimeError('Invalid pole energy')
    return dict(energy_fraction=fraction,pole_rate_rad_s=omega,
                phi_rad=math.atan2(math.sin(phi),math.cos(phi)),
                window_ms=(window[-1]['at_ns']-window[0]['at_ns'])/1e6,
                source_ns=latest)

def tapered_target(current,endpoint,energy_fraction,direction):
    if not math.isfinite(energy_fraction) or energy_fraction<0:
        raise ValueError('Invalid energy fraction')
    if direction not in (-1,1):raise ValueError('Invalid direction')
    fraction=max(0.,min(1.,(1-energy_fraction)/(1-FULL_UNTIL)))
    blend=max(0.,min(1.,(energy_fraction-TRANSITION_START)/(FINAL_ENERGY_THRESHOLD-TRANSITION_START)))
    blend=blend*blend*(3-2*blend)
    fraction=(1-blend)*fraction+blend*min(fraction,FINAL_MOVE_CAP)
    if direction*(endpoint-current)<=0:return None,0.
    target=current+round((endpoint-current)*fraction)
    if abs(target-current)<MIN_COUNTS:return None,fraction
    return target,fraction
