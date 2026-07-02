"""Breakaway-torque analysis of telemetry logs.

Synthetic telemetry rows (no hardware, no files) exercise the parser and the
per-direction breakaway estimate used by ``python -m robstride_gui.backdrive``.
"""

from __future__ import annotations

from robstride_gui.backdrive import analyze, parse_telemetry
from robstride_gui.datalog import COLUMNS


def _row(device_id: int, velocity_rpm: float, torque_nm: float,
         ts: str = "2026-07-01T10:00:00.000") -> str:
    values = {name: "" for name in COLUMNS}
    values.update(timestamp=ts, device_id=str(device_id), position_rad="0.0",
                  velocity_rpm=f"{velocity_rpm}", torque_nm=f"{torque_nm}",
                  temperature_c="30.0")
    return "\t".join(values[name] for name in COLUMNS)


def _log(rows: list[str]) -> str:
    return "\n".join(["\t".join(COLUMNS), *rows]) + "\n"


def test_parse_skips_blank_and_malformed_rows():
    text = _log([_row(4, 0.0, 0.5), "", "garbage\trow", _row(4, 5.0, 1.0)])

    samples = parse_telemetry(text)

    assert len(samples) == 2
    assert samples[0].device_id == 4
    assert samples[1].velocity_rpm == 5.0


def test_parse_raises_when_required_columns_missing():
    text = "timestamp\tdevice_id\n2026-07-01T10:00:00.000\t4\n"

    try:
        parse_telemetry(text)
    except ValueError as exc:
        assert "torque_nm" in str(exc)
    else:
        raise AssertionError("expected ValueError for missing columns")


def test_breakaway_is_peak_torque_held_while_still():
    # Torque ramps 0.5 -> 2.0 Nm while still, then the shaft breaks free at 2.5.
    rows = [
        _row(4, 0.0, 0.5), _row(4, 0.0, 1.0), _row(4, 0.0, 2.0),
        _row(4, 8.0, 2.5), _row(4, 12.0, 2.2),
    ]
    results = analyze(parse_telemetry(_log(rows)), motion_rpm=1.0)

    assert len(results) == 1
    r = results[0]
    assert r.direction == "positive"
    assert r.moved is True
    assert r.breakaway_nm == 2.0            # last still sample, not the moving one
    assert r.kinetic_nm == 2.35             # median of moving |torque| (2.5, 2.2)


def test_positive_and_negative_directions_reported_separately():
    rows = [
        _row(4, 0.0, 1.0), _row(4, 5.0, 1.5),      # +dir breaks free at 1.0
        _row(4, 0.0, -1.0), _row(4, 0.0, -2.0), _row(4, -6.0, -2.4),  # -dir at 2.0
    ]
    results = {r.direction: r for r in analyze(parse_telemetry(_log(rows)))}

    assert results["positive"].breakaway_nm == 1.0
    assert results["negative"].breakaway_nm == 2.0
    assert results["negative"].moved is True


def test_never_moved_reports_max_torque_and_not_moved():
    rows = [_row(4, 0.0, 1.0), _row(4, 0.0, 3.0), _row(4, 0.0, 5.0)]

    r = analyze(parse_telemetry(_log(rows)), motion_rpm=1.0)[0]

    assert r.moved is False
    assert r.breakaway_nm == 5.0
    assert r.kinetic_nm is None
