"""Command validation independent of ROS and physical hardware."""

from enum import IntEnum
from numbers import Integral


class CommandKind(IntEnum):
    ANGLE = 0
    SPEED = 1
    FORCE = 2
    CURRENT_LIMIT = 3
    MODE = 4


def validate_values(kind, values):
    """Validate the ROS command shape; the driver owns all hardware limits."""
    if isinstance(kind, bool) or not isinstance(kind, Integral):
        raise ValueError('kind must be an integer CommandKind')
    try:
        kind = CommandKind(kind)
    except ValueError as exc:
        raise ValueError(f'unknown command kind: {kind}') from exc
    if isinstance(values, (str, bytes)):
        raise ValueError('values must contain exactly 13 integers')
    try:
        values = list(values)
    except TypeError as exc:
        raise ValueError('values must contain exactly 13 integers') from exc
    if len(values) != 13:
        raise ValueError(f'expected 13 values, got {len(values)}')
    if any(isinstance(value, bool) or not isinstance(value, Integral) for value in values):
        raise ValueError('values must be integers, not floats or booleans')
    if any(not -32768 <= value <= 32767 for value in values):
        raise ValueError('values must fit int16 [-32768, 32767]')
    return [int(value) for value in values]
