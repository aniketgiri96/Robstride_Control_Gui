"""Unit tests for the safety envelope and per-motor calibration (no hardware).

These pin down the *last line of defense* before a command reaches the motor:
soft position bounds, velocity/current/torque caps, the E-stop latch, and the
user<->raw frame conversion. A regression here can drive real hardware past a
configured bound, so the clamps are tested at and beyond every edge.
"""

from __future__ import annotations

import math

import pytest

from robstride_gui import protocol as proto
from robstride_gui.safety import (
    Calibration,
    SafetyLimits,
    SafetyState,
)


# --- SafetyLimits.for_model defaults --------------------------------------------


def test_for_model_derives_conservative_defaults_from_datasheet():
    lim = proto.model_limits("rs-04")
    s = SafetyLimits.for_model("rs-04")
    # position is symmetric about zero at the full model span
    assert s.position_min == pytest.approx(-lim["position"])
    assert s.position_max == pytest.approx(lim["position"])
    # velocity/torque are intentionally gentled to a fraction of the absolute max
    assert s.velocity_max == pytest.approx(0.6 * lim["velocity"])
    assert s.torque_max == pytest.approx(0.5 * lim["torque"])
    # current is a fixed conservative cap, gains pass through at model max
    assert s.current_max == pytest.approx(10.0)
    assert s.kp_max == pytest.approx(lim["kp"])
    assert s.kd_max == pytest.approx(lim["kd"])


def test_for_model_unknown_falls_back_to_rs04():
    assert SafetyLimits.for_model("nope") == SafetyLimits.for_model("rs-04")


def test_for_model_honours_explicit_position_span():
    s = SafetyLimits.for_model("rs-04", position_span=1.5)
    assert s.position_min == pytest.approx(-1.5)
    assert s.position_max == pytest.approx(1.5)


def test_limits_with_returns_modified_copy_without_mutating_original():
    base = SafetyLimits.for_model("rs-04")
    tightened = base.with_(velocity_max=2.0)
    assert tightened.velocity_max == pytest.approx(2.0)
    assert base.velocity_max != pytest.approx(2.0)  # original untouched (frozen)


# --- position clamp (asymmetric bounds) -----------------------------------------


def _state(**limits) -> SafetyState:
    return SafetyState(limits=SafetyLimits(**limits))


def test_clamp_position_holds_within_bounds():
    st = _state(position_min=-1.0, position_max=2.0)
    assert st.clamp_position(0.5) == pytest.approx(0.5)
    assert st.clamp_position(-1.0) == pytest.approx(-1.0)   # on the edge
    assert st.clamp_position(2.0) == pytest.approx(2.0)


def test_clamp_position_clamps_beyond_bounds():
    st = _state(position_min=-1.0, position_max=2.0)
    assert st.clamp_position(5.0) == pytest.approx(2.0)
    assert st.clamp_position(-3.0) == pytest.approx(-1.0)


def test_clamp_position_none_bound_disables_that_side():
    st = _state(position_min=None, position_max=1.0)
    assert st.clamp_position(-1e9) == pytest.approx(-1e9)   # no lower bound
    assert st.clamp_position(2.0) == pytest.approx(1.0)


# --- symmetric (abs) caps: velocity / current / torque --------------------------


@pytest.mark.parametrize(
    "method,cap_field",
    [
        ("clamp_velocity", "velocity_max"),
        ("clamp_current", "current_max"),
        ("clamp_torque", "torque_max"),
    ],
)
def test_abs_caps_clamp_both_signs(method, cap_field):
    st = _state(**{cap_field: 4.0})
    clamp = getattr(st, method)
    assert clamp(3.0) == pytest.approx(3.0)
    assert clamp(10.0) == pytest.approx(4.0)
    assert clamp(-10.0) == pytest.approx(-4.0)
    assert clamp(0.0) == pytest.approx(0.0)


def test_abs_cap_uses_magnitude_even_if_configured_negative():
    # a mistakenly negative cap must still bound symmetrically (abs is applied)
    st = _state(velocity_max=-4.0)
    assert st.clamp_velocity(10.0) == pytest.approx(4.0)
    assert st.clamp_velocity(-10.0) == pytest.approx(-4.0)


def test_abs_cap_none_passes_through():
    st = _state(velocity_max=None, current_max=None, torque_max=None)
    assert st.clamp_velocity(1e6) == pytest.approx(1e6)
    assert st.clamp_current(-1e6) == pytest.approx(-1e6)
    assert st.clamp_torque(1e6) == pytest.approx(1e6)


# --- gain clamps (lower-bounded at zero) ----------------------------------------


def test_kp_kd_clamp_to_zero_floor_and_model_ceiling():
    st = _state(kp_max=500.0, kd_max=5.0)
    assert st.clamp_kp(-1.0) == pytest.approx(0.0)     # negative stiffness forbidden
    assert st.clamp_kp(250.0) == pytest.approx(250.0)
    assert st.clamp_kp(9999.0) == pytest.approx(500.0)
    assert st.clamp_kd(-1.0) == pytest.approx(0.0)
    assert st.clamp_kd(9999.0) == pytest.approx(5.0)


# --- E-stop latch ----------------------------------------------------------------


def test_estop_latch_engages_and_clears():
    st = SafetyState(limits=SafetyLimits.for_model("rs-04"))
    assert st.estop is False
    st.engage_estop()
    assert st.estop is True
    st.clear_estop()
    assert st.estop is False


def test_estop_does_not_alter_clamp_bounds():
    # the latch is a separate gate; clamps themselves stay value-stable
    st = _state(position_min=-1.0, position_max=1.0)
    before = st.clamp_position(5.0)
    st.engage_estop()
    assert st.clamp_position(5.0) == pytest.approx(before)


# --- Calibration: user <-> raw frame conversion ---------------------------------


def test_calibration_default_is_identity():
    c = Calibration()
    for v in (-math.pi, -1.0, 0.0, 0.25, math.pi):
        assert c.pos_to_raw(v) == pytest.approx(v)
        assert c.pos_from_raw(v) == pytest.approx(v)
        assert c.signed_to_raw(v) == pytest.approx(v)


@pytest.mark.parametrize("direction", [1, -1])
@pytest.mark.parametrize("offset", [0.0, 0.5, -1.25])
@pytest.mark.parametrize("user", [-2.0, 0.0, 1.7])
def test_calibration_position_roundtrip(direction, offset, user):
    c = Calibration(direction=direction, offset=offset)
    assert c.pos_from_raw(c.pos_to_raw(user)) == pytest.approx(user)


def test_calibration_offset_only_affects_position_not_signed():
    c = Calibration(direction=1, offset=0.5)
    assert c.pos_to_raw(0.0) == pytest.approx(0.5)        # zero trim applied
    assert c.pos_from_raw(0.5) == pytest.approx(0.0)
    # velocity/current/torque use the direction-only mapping (no offset shift)
    assert c.signed_to_raw(3.0) == pytest.approx(3.0)
    assert c.signed_from_raw(3.0) == pytest.approx(3.0)


def test_calibration_invert_flips_sign_keeps_offset_in_raw_frame():
    c = Calibration(direction=-1, offset=0.5)
    # raw = user*dir + offset
    assert c.pos_to_raw(1.0) == pytest.approx(-0.5)
    # signed mapping is pure sign flip, no offset
    assert c.signed_to_raw(2.0) == pytest.approx(-2.0)
    assert c.signed_from_raw(c.signed_to_raw(2.0)) == pytest.approx(2.0)


def test_calibration_two_motors_mirror_each_other():
    left = Calibration(direction=1)
    right = Calibration(direction=-1)
    assert left.pos_to_raw(1.0) == pytest.approx(1.0)
    assert right.pos_to_raw(1.0) == pytest.approx(-1.0)


# --- integration: clamp then convert (the order the worker applies them) --------


def test_clamp_then_calibrate_keeps_command_within_raw_bounds():
    st = _state(position_min=-1.0, position_max=1.0)
    c = Calibration(direction=-1, offset=0.0)
    # user asks for 5.0; safety clamps to +1.0, calibration mirrors to -1.0 raw
    clamped = st.clamp_position(5.0)
    assert clamped == pytest.approx(1.0)
    assert c.pos_to_raw(clamped) == pytest.approx(-1.0)
