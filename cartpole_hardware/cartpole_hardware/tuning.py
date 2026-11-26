"""Pure scoring, settling and recovery policy for bounded physical LQR search."""
import math

RECOVERABLE_FAULTS=frozenset((
    'hybrid: Swing-up timed out before LQR capture',
    'hybrid: Lost LQR capture; trial aborted',
    'hybrid: Balance settling/hold timeout',
))


def recoverable(fault, status):
    return (fault in RECOVERABLE_FAULTS and status.get('enabled') is False
            and status.get('stop_confirmed') is True
            and status.get('sensor_fresh') is True and not status.get('sensor_error')
            and not status.get('left_blocked') and not status.get('right_blocked'))


def settled_down(rows, calibrated=True, reference_count=None):
    """Require >=2 seconds of fresh, nonduplicate raw telemetry observations."""
    if len(rows)<80:return False
    times=[q['host_monotonic_ns']/1e9 for q in rows]
    if times[-1]-times[0]<2 or any(not 0<b-a<=.060 for a,b in zip(times,times[1:])):
        return False
    angles=[q['pole_rad'] for q in rows]
    velocities=[q['cart_m_s'] for q in rows]
    rates=[q['pole_rad_s'] for q in rows]
    if not all(math.isfinite(x) for x in angles+velocities+rates):return False
    if max(abs(x) for x in velocities)>.001 or max(abs(x) for x in rates)>.10:return False
    if calibrated and max(abs(abs(x)-math.pi) for x in angles)>.035:return False
    # Relative angle spread <=2 encoder counts, accounting for the +/-pi seam.
    differences=[(x-angles[0]+math.pi)%(2*math.pi)-math.pi for x in angles]
    if max(differences)-min(differences)>2*2*math.pi/2000+1e-8:return False
    if not calibrated:
        if reference_count is None:return False
        counts=[q['encoder_count'] for q in rows]
        delta=[((c-reference_count+1000)%2000)-1000 for c in counts]
        if max(abs(d) for d in delta)>10:return False
    return True


def score_hold(rows):
    """Score the last fixed five seconds of balance, using raw measurements.

    Never score failed captures as wins. Lower is better. Data gaps invalidate
    comparison, rather than making sparse/missing observations appear quieter.
    """
    rows=[r for r in rows if (r.get('hybrid') or {}).get('phase')=='balance']
    if not rows:raise ValueError('No balance telemetry')
    end=rows[-1]['host_monotonic_ns']
    rows=[r for r in rows if end-r['host_monotonic_ns']<=5_000_000_000]
    times=[r['host_monotonic_ns']/1e9 for r in rows]
    if len(rows)<200 or times[-1]-times[0]<4.8:
        raise ValueError('Insufficient five-second balance record')
    gaps=[b-a for a,b in zip(times,times[1:])]
    if any(not 0<g<=.060 for g in gaps):raise ValueError('Balance recording gaps/out-of-order data')
    keys=('pole_rad','pole_rad_s','cart_m','cart_m_s','command_m_s')
    if any(not math.isfinite(r[k]) for r in rows for k in keys):
        raise ValueError('Non-finite tuning score input')
    if max(abs(r['pole_rad']) for r in rows)>.06 or max(abs(r['cart_m']-.0735) for r in rows)>.010:
        raise ValueError('Raw measured hold departed from tuning verification window')
    def rms(key,target=0.):return math.sqrt(sum((r[key]-target)**2 for r in rows)/len(rows))
    angle=rms('pole_rad');rate=rms('pole_rad_s');cart=rms('cart_m',.0735)
    velocity=rms('cart_m_s');command=rms('command_m_s')
    saturation=sum(abs(r['command_m_s'])>=.095 for r in rows)/len(rows)
    cost=(angle/.035)**2+.25*(rate/.35)**2+(cart/.005)**2
    cost+=.25*(velocity/.015)**2+.1*(command/.1)**2+2*saturation
    return dict(cost=cost,angle_rms_rad=angle,rate_rms_rad_s=rate,
                cart_rms_m=cart,velocity_rms_m_s=velocity,command_rms_m_s=command,
                saturation_fraction=saturation,samples=len(rows),span_s=times[-1]-times[0])


def choose_best(trials):
    valid=[t for t in trials if t.get('success') and t.get('score')
           and math.isfinite(t['score'].get('cost',math.inf))]
    return min(valid,key=lambda t:t['score']['cost']) if valid else None
