"""Hardware-independent conversions, framing and motion interlocks."""
import math
from dataclasses import dataclass


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('non-finite value')
    return value


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def parse_sensor(line):
    fields = line.strip().split(',')
    if len(fields) != 9 or fields[0] != 'CP1':
        raise ValueError('invalid sensor frame')
    seq, us, count, left, right, z, invalid, boot = map(int, fields[1:])
    if not (0 <= seq <= 0xffffffff and 0 <= us <= 0xffffffff
            and -2147483648 <= count <= 2147483647
            and left in (0, 1) and right in (0, 1) and z in (0, 1)
            and invalid >= 0 and boot == 1):
        raise ValueError('sensor values out of range')
    return seq, us, count, bool(left), bool(right), z, invalid


@dataclass
class Geometry:
    circumference: float
    ratio: float
    counts: int
    sign: int

    def __post_init__(self):
        if not (finite(self.circumference) > 0 and finite(self.ratio) > 0
                and self.counts > 0 and self.sign in (-1, 1)):
            raise ValueError('invalid drive geometry')

    def position(self, counts):
        return self.sign * counts * self.circumference / (self.counts * self.ratio)

    def rpm(self, velocity):
        return self.sign * finite(velocity) * 60 * self.ratio / self.circumference


def guarded_velocity(v, x, left, right, lower, upper, vmax, braking, margin):
    """Stop before soft limits; blocked hardware switches always fault upstream."""
    v = max(-vmax, min(vmax, finite(v)))
    if left and v < 0 or right and v > 0:
        return 0.0
    distance = upper - x if v > 0 else x - lower
    # Velocity envelope assumes validated minimum physical braking deceleration.
    allowed = math.sqrt(2 * braking * max(0.0, distance - margin))
    return math.copysign(min(abs(v), allowed), v)
