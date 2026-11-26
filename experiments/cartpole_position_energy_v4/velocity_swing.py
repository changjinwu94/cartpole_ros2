"""Phase-locked swing-up using signed cart velocity in m/s. No ROS imports."""
import math
from dataclasses import dataclass

from energy_core import INERTIA, MG
from phase_core import DownCrossings, PhasePolicy
from profile_core import SCALE, targets
from state_push import state_push_target


@dataclass(frozen=True)
class Settings:
    fraction: float = 0.32
    energy: float = 0.83
    duration: float = 8.0
    speed: float = 0.08
    acceleration: float = 1.0

    def __post_init__(self):
        for name, high in (("fraction", 1), ("energy", 1), ("duration", 20),
                           ("speed", 0.275), ("acceleration", 3.5)):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0 < value <= high:
                raise ValueError(f"{name} must be finite, positive and at most {high}")


# Match the commissioned velocity envelope and the position experiment's inner travel window.
BRAKING = 3.5            # m/s², minimum braking used by guarded_velocity
INNER_MARGIN = 0.005     # m, reserve inside the trial boundary
MAX_GAP_NS = 80_000_000  # 50 Hz telemetry, fail on a prolonged gap
UPRIGHT = math.radians(12)
STARTUP_PEAK = math.radians(3)
STARTUP_RETURN = math.radians(0.5)
RESPONSE_ALLOWANCE_S = 0.06
PHASE_ADVANCE_S = 0.06
ADVANCE_MIN_ENERGY = 0.65
ADVANCE_MAX_ENERGY = 0.78


def delayed_speed_cap(braking, distance):
    """Solve distance = speed * delay + speed² / (2 * braking)."""
    lag_speed = braking * RESPONSE_ALLOWANCE_S
    return math.sqrt(lag_speed**2 + 2 * braking * distance) - lag_speed
MAX_FEEDBACK_SPEED = 4000 * 0.04 / (60 * 9.7)  # commissioned motor RPM limit in cart m/s


class Swing:
    """Choose a stroke at a down-crossing, then stream speed toward its endpoint."""

    def __init__(self, start_x, start_ns, settings):
        if not math.isfinite(start_x) or not 0.070 <= start_x <= 0.077:
            raise ValueError("Start outside the centred reference window")
        self.start_x = start_x
        self.lower = start_x - 0.035
        self.upper = start_x + 0.035
        self.settings = settings
        self.crossings = DownCrossings(0)
        self.phase = PhasePolicy(start_ns)
        self.previous_stamp = None
        self.direction = 0
        self.endpoint = start_x
        self.estimated_x = start_x
        self.previous_speed = None
        self.sent_final = False
        self.strokes = 0
        self.startup_peak = 0.0
        self.last_request = 0.0
        self.closest = math.pi
        self.entry = None
        self.outcome = None

    def update(self, now_ns, q):
        """Consume one distinct telemetry sample. Returns (velocity request, optional decision)."""
        stamp = q["host_monotonic_ns"]
        if not 0 <= now_ns - stamp <= MAX_GAP_NS:
            raise RuntimeError("Stale cart telemetry")
        previous = self.previous_stamp
        if previous is not None and not 0 < stamp - previous <= MAX_GAP_NS:
            raise RuntimeError("Invalid telemetry interval")
        self.previous_stamp = stamp
        speed = q["cart_m_s"]
        x, angle, rate = q["cart_m"], q["pole_rad"], q["pole_rad_s"]
        if not all(map(math.isfinite, (x, angle, rate, speed))):
            raise RuntimeError("Nonfinite cart/pole telemetry")
        if abs(speed) > MAX_FEEDBACK_SPEED:
            raise RuntimeError("Measured cart speed exceeds motor RPM limit")
        if not self.lower <= x <= self.upper:
            raise RuntimeError("Cart outside swing travel window")

        if previous is not None:
            elapsed = (stamp - previous) / 1e9
            self.estimated_x += 0.5 * (self.previous_speed + speed) * elapsed
        self.previous_speed = speed

        self.closest = min(self.closest, abs(angle))
        # ROS reports zero at upright. The phase and energy helpers use zero at down.
        phi = angle % math.tau - math.pi
        energy = (0.5 * INERTIA * rate**2 + MG * (1 - math.cos(phi))) / (2 * MG)
        if not math.isfinite(energy):
            raise RuntimeError("Invalid pole energy")
        estimate = dict(energy_fraction=energy, pole_rate_rad_s=rate, phi_rad=phi, source_ns=stamp)
        if abs(angle) <= UPRIGHT:
            self.entry = estimate
            self.outcome = "upright_entry"
            return 0.0, None

        crossing = self.crossings.update(phi * 2000 / math.tau, stamp)
        direction, reason = self.phase.choose(now_ns, [crossing] if crossing else [])
        # The first rightward push can send the pole to the positive turning
        # point before the 5° bottom detector arms. Start the return stroke
        # while the pole is heading left and still above the bottom.
        if self.strokes == 1:
            self.startup_peak = max(self.startup_peak, phi)
            if (direction is None and phi > 0 and rate < -0.15
                    and self.startup_peak >= STARTUP_PEAK
                    and self.startup_peak - phi >= STARTUP_RETURN
                    and now_ns - self.phase.last_command >= 150_000_000):
                direction, reason = self.phase.begin_stroke(now_ns, -1, "startup_turnaround")
        predicted_crossing_s = None
        if (direction is None and crossing is None and self.strokes >= 3
                and not self.sent_final and ADVANCE_MIN_ENERGY <= energy < min(ADVANCE_MAX_ENERGY, self.settings.energy)
                and self.crossings.side * phi > 0 and phi * rate < 0
                and now_ns - self.phase.last_command >= 150_000_000):
            lead = -phi / rate
            approaching_direction = -self.crossings.side
            if 0 < lead <= PHASE_ADVANCE_S and approaching_direction != self.phase.last_target:
                endpoint = self.start_x + targets(0)[0 if approaching_direction > 0 else 1] * SCALE
                if approaching_direction * (endpoint - self.estimated_x) > 0:
                    predicted_crossing_s = lead
                    direction, reason = self.phase.begin_stroke(now_ns, approaching_direction, "phase_advance")
        decision = None
        if direction is not None:
            if self.sent_final:
                self.outcome = "final_push_returned_without_upright_entry"
                return 0.0, None
            if self.strokes >= 40:
                raise RuntimeError("Swing stroke limit reached")
            current = round((self.estimated_x - self.start_x) / SCALE)
            endpoint = targets(0)[0 if direction > 0 else 1]
            if crossing is not None and reason == "down_crossing" and energy >= self.settings.energy:
                if direction * (endpoint - current) <= 0:
                    raise RuntimeError("Final stroke endpoint behind the cart")
                fraction = self.settings.fraction
                goal = current + round(fraction * (endpoint - current))
                self.sent_final = True
                reason = "reduced_final_push"
            else:
                goal, fraction = state_push_target(current, endpoint, rate, direction)
            self.direction = direction
            self.endpoint = self.estimated_x if goal is None else self.start_x + goal * SCALE
            self.strokes += 1
            decision = dict(kind="energy_decision", host_monotonic_ns=now_ns, direction=direction,
                            reason=reason, move_fraction=fraction, current_cart_m=x,
                            estimated_cart_m=self.estimated_x,
                            stroke_end_m=self.endpoint, predicted_crossing_s=predicted_crossing_s, **estimate)

        # Plan braking for the remaining stroke and stream a slew-limited signed speed.
        braking = min(BRAKING, self.settings.acceleration)
        distance = max(0.0, self.direction * (self.endpoint - self.estimated_x))
        desired = self.direction * min(self.settings.speed, delayed_speed_cap(braking, distance))
        # First sample establishes the command stream at zero. Use at most two nominal ticks.
        dt = 0.0 if previous is None else min((stamp - previous) / 1e9, 0.04)
        step = self.settings.acceleration * dt
        request = self.last_request + max(-step, min(step, desired - self.last_request))
        # Recheck the direction AFTER the slew, including while changing direction.
        rail_distance = (self.upper - x) if request > 0 else (x - self.lower)
        rail_cap = delayed_speed_cap(braking, max(0.0, rail_distance - INNER_MARGIN))
        self.last_request = math.copysign(min(abs(request), rail_cap), request)
        return self.last_request, decision
