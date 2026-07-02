"""Backdrivability analysis for telemetry logs.

Reads a telemetry ``.txt`` written by :class:`~robstride_gui.datalog.TelemetryLogger`
and estimates, per motor and per direction, the **breakaway torque**: the largest
feed-forward torque the shaft withstood while still, just before it started to
rotate. Lower breakaway torque = more backdrivable.

The intended capture (see the SOP backdrive procedure): MIT mode, Kp=Kd=0, then
ramp the "Assist τ" slider up until the shaft breaks free, in each direction,
with "Record log" on. This tool then pulls the number out of the log so you do
not have to eyeball the live plot.

Usage::

    python -m robstride_gui.backdrive [LOGFILE] [--motion-rpm N] [--model rs-04]

With no LOGFILE, the newest ``telemetry-*.txt`` in the default log dir is used.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .datalog import COLUMNS, default_log_dir
from .protocol import DEFAULT_MODEL, model_limits

#: Torques with |value| below this (Nm) are treated as "no command" and ignored
#: when splitting a run into its positive/negative assist directions.
_ZERO_TORQUE = 1e-6


@dataclass(frozen=True)
class Sample:
    timestamp: str
    device_id: int
    position: float
    velocity_rpm: float
    torque_nm: float


@dataclass(frozen=True)
class DirectionResult:
    device_id: int
    direction: str                 # "positive" | "negative"
    moved: bool
    breakaway_nm: float            # max |torque| held while still before motion
    kinetic_nm: Optional[float]    # median |torque| once moving (None if never moved)
    onset_timestamp: Optional[str]
    samples: int


def parse_telemetry(text: str) -> list[Sample]:
    """Parse the tab-separated telemetry text into time-ordered samples.

    Columns are looked up by header name (robust to column reordering); rows that
    are blank or malformed are skipped rather than aborting the whole file.
    """
    lines = [ln for ln in text.splitlines() if ln.strip()]
    if not lines:
        return []
    header = lines[0].split("\t")
    idx = {name: header.index(name) for name in COLUMNS if name in header}
    required = ("device_id", "position_rad", "velocity_rpm", "torque_nm")
    missing = [name for name in required if name not in idx]
    if missing:
        raise ValueError(f"telemetry file missing required columns: {missing}")
    out: list[Sample] = []
    for ln in lines[1:]:
        cols = ln.split("\t")
        try:
            out.append(Sample(
                timestamp=cols[idx["timestamp"]] if "timestamp" in idx else "",
                device_id=int(cols[idx["device_id"]]),
                position=float(cols[idx["position_rad"]]),
                velocity_rpm=float(cols[idx["velocity_rpm"]]),
                torque_nm=float(cols[idx["torque_nm"]]),
            ))
        except (ValueError, IndexError):
            continue  # partial/garbled row (e.g. logging interrupted) -> skip
    return out


def _median(xs: list[float]) -> Optional[float]:
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return None
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])


def _analyze_one(samples: list[Sample], direction: str,
                 device_id: int, motion_rpm: float) -> DirectionResult:
    """Analyze one motor's samples for one assist direction (time-ordered)."""
    onset = next((i for i, s in enumerate(samples)
                  if abs(s.velocity_rpm) >= motion_rpm), None)
    if onset is None:
        # Never broke free within the recorded torque range.
        breakaway = max((abs(s.torque_nm) for s in samples), default=0.0)
        return DirectionResult(device_id, direction, False, breakaway,
                               None, None, len(samples))
    still = samples[:onset] or samples[:1]
    breakaway = max(abs(s.torque_nm) for s in still)
    moving = [abs(s.torque_nm) for s in samples[onset:]
              if abs(s.velocity_rpm) >= motion_rpm]
    return DirectionResult(device_id, direction, True, breakaway,
                           _median(moving), samples[onset].timestamp,
                           len(samples))


def analyze(samples: list[Sample], motion_rpm: float = 1.0) -> list[DirectionResult]:
    """Group samples by motor and assist direction, return breakaway results.

    ``motion_rpm`` is the velocity above which the shaft is considered "moving".
    """
    device_ids = sorted({s.device_id for s in samples})
    results: list[DirectionResult] = []
    for did in device_ids:
        dev = [s for s in samples if s.device_id == did]
        pos = [s for s in dev if s.torque_nm > _ZERO_TORQUE]
        neg = [s for s in dev if s.torque_nm < -_ZERO_TORQUE]
        if pos:
            results.append(_analyze_one(pos, "positive", did, motion_rpm))
        if neg:
            results.append(_analyze_one(neg, "negative", did, motion_rpm))
    return results


def _verdict(fraction: float) -> str:
    """Map breakaway-torque-as-fraction-of-rated to a human verdict."""
    if fraction < 0.025:
        return "backdrivable"
    if fraction < 0.07:
        return "stiff (typical geared)"
    return "NOT hand-backdrivable"


def format_report(results: list[DirectionResult], rated_torque: float,
                  model: str, motion_rpm: float, path: Path) -> str:
    lines = [
        f"Backdrivability report: {path}",
        f"Rated torque ({model}): {rated_torque:.1f} Nm    "
        f"motion threshold: {motion_rpm:g} rpm",
        "",
    ]
    if not results:
        lines.append("No assist-torque samples found. Ramp the 'Assist τ' slider "
                     "(MIT, Kp=Kd=0) while recording, then re-run.")
        return "\n".join(lines)
    for r in results:
        frac = r.breakaway_nm / rated_torque if rated_torque else 0.0
        if r.moved:
            kin = f"{r.kinetic_nm:.2f}" if r.kinetic_nm is not None else "n/a"
            lines.append(
                f"Motor {r.device_id}  {r.direction:>8}: "
                f"breakaway {r.breakaway_nm:6.2f} Nm ({frac*100:4.1f}% of rated) "
                f"-> {_verdict(frac)}   kinetic {kin} Nm   onset {r.onset_timestamp}")
        else:
            lines.append(
                f"Motor {r.device_id}  {r.direction:>8}: "
                f"did NOT move up to {r.breakaway_nm:.2f} Nm "
                f"({frac*100:.1f}% of rated) -> increase assist τ or raise the "
                f"torque cap and retry")
    return "\n".join(lines)


def _newest_log() -> Optional[Path]:
    log_dir = default_log_dir()
    logs = sorted(log_dir.glob("telemetry-*.txt"))
    return logs[-1] if logs else None


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Estimate motor breakaway (backdrive) torque from a telemetry log.")
    parser.add_argument("logfile", nargs="?", type=Path,
                        help="telemetry .txt (default: newest in the log dir)")
    parser.add_argument("--motion-rpm", type=float, default=1.0,
                        help="velocity (rpm) above which the shaft counts as moving")
    parser.add_argument("--model", default=DEFAULT_MODEL,
                        help="motor model for the rated-torque reference")
    args = parser.parse_args(argv)

    path = args.logfile or _newest_log()
    if path is None:
        print("No telemetry log found. Record a run first (Record log toggle), "
              f"or pass a path. Log dir: {default_log_dir()}", file=sys.stderr)
        return 2
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        print(f"Cannot read {path}: {exc}", file=sys.stderr)
        return 2
    try:
        samples = parse_telemetry(text)
    except ValueError as exc:
        print(f"Bad telemetry file: {exc}", file=sys.stderr)
        return 2

    rated = model_limits(args.model)["torque"]
    results = analyze(samples, motion_rpm=args.motion_rpm)
    print(format_report(results, rated, args.model, args.motion_rpm, Path(path)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
