"""Safety envelope for motor commands.

Centralizes the limits the GUI enforces *before* anything reaches the motor:
soft position bounds, velocity / current / torque caps, and a global E-stop
latch. Commands are clamped (not rejected) so a slider can never drive the
motor past a configured bound, and when E-stop is engaged every motion request
collapses to "hold/zero".
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from . import protocol as proto


@dataclass(frozen=True)
class SafetyLimits:
    """Per-motor soft limits. ``None`` disables that particular bound."""

    position_min: float | None = None    # rad
    position_max: float | None = None    # rad
    velocity_max: float | None = None    # rad/s (absolute)
    current_max: float | None = None     # A (absolute)
    torque_max: float | None = None      # Nm (absolute)
    kp_max: float | None = None
    kd_max: float | None = None

    @classmethod
    def for_model(cls, model: str, position_span: float | None = None) -> "SafetyLimits":
        """Conservative defaults derived from the model's datasheet ranges.

        Defaults cap velocity/current/torque at a fraction of the absolute
        maximum so first power-on is gentle; the user can widen them in the UI.
        """
        lim = proto.model_limits(model)
        span = position_span if position_span is not None else lim["position"]
        return cls(
            position_min=-span,
            position_max=span,
            velocity_max=0.6 * lim["velocity"],
            # 25 A current ceiling: the 10 A default throttled rs-04 torque to
            # ~15 Nm (torque is current-limited well before the torque_max cap).
            # Raised to let torque climb toward torque_max; more current means
            # more heat/force, so watch temperature when running near it.
            current_max=25.0,
            torque_max=0.5 * lim["torque"],
            kp_max=lim["kp"],
            kd_max=lim["kd"],
        )

    def with_(self, **changes) -> "SafetyLimits":
        """Return a copy with ``changes`` applied (immutable update)."""
        return replace(self, **changes)


def _clamp(value: float, lo: float | None, hi: float | None) -> float:
    if lo is not None and value < lo:
        return lo
    if hi is not None and value > hi:
        return hi
    return value


def _clamp_abs(value: float, cap: float | None) -> float:
    if cap is None:
        return value
    return _clamp(value, -abs(cap), abs(cap))


@dataclass
class Calibration:
    """Per-motor frame conversion between the *user* frame and the *raw* motor.

    ``direction`` is +1 or -1 (an "invert direction" toggle) and ``offset`` is a
    software zero trim in radians, expressed in the raw motor frame. The two
    conversions are exact inverses for ``direction in (+1, -1)`` since
    ``direction * direction == 1``::

        raw  = user * direction + offset
        user = (raw - offset) * direction
    """

    direction: int = 1     # +1 normal, -1 inverted
    offset: float = 0.0    # rad, raw frame

    def pos_to_raw(self, user: float) -> float:
        return user * self.direction + self.offset

    def pos_from_raw(self, raw: float) -> float:
        return (raw - self.offset) * self.direction

    def signed_to_raw(self, user: float) -> float:
        """Direction-only mapping for velocity / current / torque (no offset)."""
        return user * self.direction

    def signed_from_raw(self, raw: float) -> float:
        return raw * self.direction


@dataclass
class SafetyState:
    """Mutable safety state shared between the UI and the worker."""

    limits: SafetyLimits
    estop: bool = False

    def engage_estop(self) -> None:
        self.estop = True

    def clear_estop(self) -> None:
        self.estop = False

    # -- clamping helpers --------------------------------------------------------

    def clamp_position(self, value: float) -> float:
        return _clamp(value, self.limits.position_min, self.limits.position_max)

    def clamp_velocity(self, value: float) -> float:
        return _clamp_abs(value, self.limits.velocity_max)

    def clamp_current(self, value: float) -> float:
        return _clamp_abs(value, self.limits.current_max)

    def clamp_torque(self, value: float) -> float:
        return _clamp_abs(value, self.limits.torque_max)

    def clamp_kp(self, value: float) -> float:
        return _clamp(value, 0.0, self.limits.kp_max)

    def clamp_kd(self, value: float) -> float:
        return _clamp(value, 0.0, self.limits.kd_max)
