"""Release gate for custom slow landing; native AUTO.LAND remains available.

The position-only descent core is testable independently. It is not yet a
released hardware landing strategy: the connected PX4 1.12.3 needs descent
intent that its bounded final position target cannot provide.
"""
import math


SLOW_LANDING_BLOCK_REASON = (
    'Slow landing is blocked: 0.2 m/s landing and native contact '
    'detection are not yet compatible with the connected PX4 1.12.3; '
    'use native AUTO.LAND for the current automatic task')


def landing_compatibility(version, land_speed, z_gain, descent_speed):
    """Explain a known mismatch, never certify an unverified firmware.

0.30 m is the existing maximum position lead. Even that upper bound is
insufficient on the connected aircraft; at the takeoff floor the final
target is only 0.15 m lower. A commanded velocity threshold is not an
actual touchdown-speed requirement.
"""
    values = (land_speed, z_gain, descent_speed)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0
               for v in values):
        return False, 'invalid or unavailable PX4 landing parameters'
    if version == 17564416:  # Reported PX4 1.12.3, normal firmware version packing.
        threshold = .9 * max(land_speed, .1)
        if threshold > .30 * z_gain:
            return False, ('PX4 1.12.3 requires a downward velocity target of '
                           'at least %.2f m/s; bounded position descent cannot '
                           'supply it (maximum %.2f m/s). No automatic landing.'
                           % (threshold, .30 * z_gain))
    return False, 'native contact detection for this firmware/configuration is unverified'
