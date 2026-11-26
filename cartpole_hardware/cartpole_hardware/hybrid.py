"""Hardware-independent energy swing-up and sampled velocity-servo LQR.

State order: x [m], v [m/s], theta [rad from upright], omega [rad/s].
Positive theta leans right: positive cart acceleration reduces theta_ddot.
No serial or ROS calls occur here. A rejected state MUST stop the drive upstream.
"""
import math
from dataclasses import dataclass

INERTIA = .00707 * .105**2 / 3 + .00064 * .100**2
FIRST_MOMENT = .00707 * .105 / 2 + .00064 * .100
GRAVITY = 9.81
SERVO_TAU = .03  # Model assumption; not a measured drive response.
DESIGN_DT = .02
# Generated with tools/lqr/design.py. Input is requested cart velocity, NOT force.
GAINS = (-4.811752121014913, -2.9651924401543646,
         -4.1368035706263875, -.32597648393109163)


def clip(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass(frozen=True)
class State:
    x: float
    v: float
    theta: float
    omega: float


class HybridController:
    """Single trial: swing -> balance -> done, or abort (no automatic retry)."""
    def __init__(self, target=.0735, lower=.050, upper=.100,
                 vmax=.1, amax=1., braking=.1, margin=.005,
                 swing_timeout=15., hold_seconds=5., gains=GAINS):
        values=(target,lower,upper,vmax,amax,braking,margin,swing_timeout,hold_seconds)
        if not all(math.isfinite(q) for q in values):
            raise ValueError('Non-finite hybrid configuration')
        if not (lower+margin < target < upper-margin and
                0 < vmax <= .1 and 0 < amax <= 1. and braking > 0 and margin > 0
                and 0 < swing_timeout <= 20 and 0 < hold_seconds <= 30):
            raise ValueError('Invalid hybrid envelope/time limits')
        if len(gains)!=4 or any(not math.isfinite(k) for k in gains):
            raise ValueError('Four finite LQR gains required')
        if any(k*b<=0 or not .70<=k/b<=1.30 for k,b in zip(gains,GAINS)):
            raise ValueError('LQR candidate outside bounded nominal gain neighborhood')
        self.gains=tuple(gains)
        self.target,self.lower,self.upper=target,lower,upper
        self.vmax,self.amax,self.braking,self.margin=vmax,amax,braking,margin
        self.swing_timeout,self.hold_seconds=swing_timeout,hold_seconds
        self.mode='idle'; self.elapsed=0.; self.balance_elapsed=0.
        self.held_seconds=0.; self.command=0.; self.capture_ticks=0
        self.energy_fraction=0.; self.raw_lqr=0.; self.reason=''
        self.kick_elapsed=0.; self.kick_active=True

    def can_capture(self, s):
        """Conservative candidate region, not a certified region of attraction."""
        raw=-sum(k*q for k,q in zip(self.gains,(s.x-self.target,s.v,s.theta,s.omega)))
        return (abs(s.theta)<=.12 and abs(s.omega)<=1.5
                and abs(s.x-self.target)<=.010 and abs(s.v)<=.050
                and abs(raw)<=self.vmax*.8)

    def start(self, s, balance_only=False):
        if self.mode not in ('idle','done','abort'):
            raise ValueError('Controller trial already running')
        self._check(s, DESIGN_DT)
        if abs(s.x-self.target)>.003 or abs(s.v)>.005:
            raise ValueError('Start centered within 3mm and stationary within 5mm/s')
        if balance_only:
            if not self.can_capture(s):
                raise ValueError('Outside LQR capture angle/rate/position/velocity window')
        elif abs(abs(s.theta)-math.pi)>.10 or abs(s.omega)>.3:
            raise ValueError('Swing start requires freely hanging, settled pole')
        self.mode='balance' if balance_only else 'swing'
        self.elapsed=self.balance_elapsed=self.held_seconds=0.
        self.command=s.v; self.capture_ticks=0; self.reason=''
        self.kick_elapsed=0.; self.kick_active=True

    def _check(self, s, dt):
        if not all(math.isfinite(q) for q in (s.x,s.v,s.theta,s.omega,dt)):
            raise ValueError('Non-finite hybrid state')
        if not .010 <= dt <= .035:
            raise ValueError(f'Hybrid control period {dt*1000:.3f}ms outside 10–35ms')
        if not self.lower<=s.x<=self.upper:
            raise ValueError('Hybrid cart outside 50–100mm experiment envelope')
        if abs(s.v)>self.vmax*1.25+.002:
            raise ValueError('Hybrid measured overspeed')
        if abs(s.omega)>35:
            raise ValueError('Hybrid pole rate exceeded 35rad/s')

    def abort(self, reason):
        self.mode='abort';self.reason=reason;self.command=0.

    def step(self, s, dt, applied_command):
        """Return velocity target; use actual last command for anti-windup.

        Node applies its existing slew and braking envelope after this method.
        Capture is sticky: loss of balance aborts, rather than re-pumping energy.
        """
        if self.mode not in ('swing','balance'):
            raise ValueError('Hybrid controller is not active')
        try:
            self._check(s,dt)
            if not math.isfinite(applied_command):
                raise ValueError('Non-finite applied velocity')
            self.elapsed+=dt
            self.energy_fraction=(.5*INERTIA*s.omega**2+
                FIRST_MOMENT*GRAVITY*(1+math.cos(s.theta)))/(2*FIRST_MOMENT*GRAVITY)
            if self.mode=='swing':
                if self.elapsed>self.swing_timeout:
                    raise ValueError('Swing-up timed out before LQR capture')
                self.capture_ticks=self.capture_ticks+1 if self.can_capture(s) else 0
                if self.capture_ticks>=1:
                    self.mode='balance';self.balance_elapsed=0.;self.held_seconds=0.
            if self.mode=='balance':
                self.balance_elapsed+=dt
                if abs(s.theta)>.18 or abs(s.omega)>2.:
                    raise ValueError('Lost LQR capture; trial aborted')
                self.raw_lqr=-sum(k*q for k,q in zip(
                    self.gains,(s.x-self.target,s.v,s.theta,s.omega)))
                command=self.raw_lqr
                # Success means continuously meeting all four hold tolerances.
                held=(abs(s.theta)<.035 and abs(s.omega)<.35
                      and abs(s.x-self.target)<.005 and abs(s.v)<.015)
                self.held_seconds=self.held_seconds+dt if held else 0.
                if self.held_seconds>=self.hold_seconds:
                    self.mode='done';self.reason='upright_hold_completed';return 0.
                if self.balance_elapsed>self.hold_seconds+10:
                    raise ValueError('Balance settling/hold timeout')
            else:
                # Energy shaping: E_dot=-h*a*omega*cos(theta). Centering terms
                # trade energy injection for bounded cart motion.
                acceleration=4.*(self.energy_fraction-1.001)*s.omega*math.cos(s.theta)
                acceleration-=20.*(s.x-self.target)+2.*s.v
                # Finite initial kick escapes the exactly-down zero-rate state.
                # Never repeatedly kick a moving pole or restart after failure.
                if self.kick_active:
                    self.kick_elapsed+=dt
                    if abs(s.omega)>.4 or self.kick_elapsed>.12:
                        self.kick_active=False
                    else:
                        acceleration+=.5
                acceleration=clip(acceleration,-self.amax,self.amax)
                command=applied_command+acceleration*dt
            command=clip(command,-self.vmax,self.vmax)
            distance=self.upper-s.x if command>0 else s.x-self.lower
            allowed=math.sqrt(2*self.braking*max(0.,distance-self.margin))
            self.command=math.copysign(min(abs(command),allowed),command)
            return self.command
        except ValueError as exc:
            self.abort(str(exc));raise

    def report(self):
        return dict(phase=self.mode,elapsed_s=self.elapsed,
                    held_s=self.held_seconds,energy_fraction=self.energy_fraction,
                    raw_lqr_m_s=self.raw_lqr,reason=self.reason,gains=list(self.gains))


class PoleObserver:
    """Nonlinear alpha-beta observer for quantized angle and measured cart speed.

    At 50Hz, a single encoder count produces 0.157rad/s in a raw difference.
    Predict angular motion using measured cart acceleration, then correct
    angle/rate from wrapped angle innovation. Reset on calibration/long gaps.
    """
    def __init__(self):
        self.theta=None;self.omega=0.;self.v=0.;self.samples=0

    def update(self, theta, v, dt, free_motion=True):
        if not all(math.isfinite(q) for q in (theta,v,dt)):
            raise ValueError('Invalid observer input')
        if self.theta is None or not .010<=dt<=.035:
            self.theta=theta;self.omega=0.;self.v=v;self.samples=1
        else:
            acceleration=(v-self.v)/dt
            def angular(angle):
                if not free_motion:
                    return 0.0  # Held pole: external hand force is unknown.
                return (FIRST_MOMENT*GRAVITY*math.sin(angle)
                    -FIRST_MOMENT*acceleration*math.cos(angle))/INERTIA
            # RK4 avoids a large Euler prediction error during fast bottom crossings.
            t,w=self.theta,self.omega
            k1t,k1w=w,angular(t)
            k2t,k2w=w+.5*dt*k1w,angular(t+.5*dt*k1t)
            k3t,k3w=w+.5*dt*k2w,angular(t+.5*dt*k2t)
            k4t,k4w=w+dt*k3w,angular(t+dt*k3t)
            predicted=t+dt*(k1t+2*k2t+2*k3t+k4t)/6
            predicted_rate=w+dt*(k1w+2*k2w+2*k3w+k4w)/6
            error=(theta-predicted+math.pi)%(2*math.pi)-math.pi
            self.theta=(predicted+.6*error+math.pi)%(2*math.pi)-math.pi
            self.omega=predicted_rate+.12*error/dt
            self.v=v;self.samples+=1
        return self.theta,self.omega
