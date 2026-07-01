"""Telemetry data logging to a plain-text file.

Logging is *opt-in*: nothing is written until :meth:`TelemetryLogger.start` is
called (the GUI wires this to a "Record log" toggle). While recording, every
feedback sample that drives the live graphs - position, velocity, torque and
temperature, plus the board power readout when available - is appended to a
separate tab-separated ``.txt`` data file so a run can be reviewed or
post-processed offline. :meth:`stop` ends the recording and closes the file.

Each :meth:`start` opens a fresh file named for the moment recording began,
under the user's config dir::

    ~/.config/robstride_gui/logs/telemetry-YYYYMMDD-HHMMSS.txt

A header row names the columns; every later row is one feedback sample. All IO
is wrapped so a logging failure (full disk, read-only path) stops the recording
rather than crashing the GUI.
"""

from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path
from typing import Optional, TextIO

from .protocol import RAD_S_TO_RPM

#: Tab-separated columns written to the data file, in order.
COLUMNS = (
    "timestamp",
    "device_id",
    "position_rad",
    "velocity_rpm",
    "torque_nm",
    "temperature_c",
    "vbus_v",
    "iq_a",
    "power_w",
    "faults",
)


def default_log_dir() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME") or os.path.join(Path.home(), ".config")
    return Path(base) / "robstride_gui" / "logs"


def _timestamped_name(now: datetime) -> str:
    return f"telemetry-{now:%Y%m%d-%H%M%S}.txt"


class TelemetryLogger:
    """Records motor feedback samples to a tab-separated ``.txt`` data file.

    Idle until :meth:`start`; while recording, each :meth:`log_status` appends a
    row. The latest board power reading is carried forward via
    :meth:`update_power` so each feedback row also records VBUS / Iq / power when
    known.
    """

    def __init__(self, log_dir: Optional[Path] = None):
        self._log_dir = Path(log_dir) if log_dir is not None else default_log_dir()
        self.path: Optional[Path] = None
        self._fh: Optional[TextIO] = None
        self._recording = False
        # device_id -> (vbus, iq, power); newest power read kept per motor.
        self._power: dict[int, tuple[float, float, float]] = {}

    @property
    def is_recording(self) -> bool:
        return self._recording

    # -- start / stop -------------------------------------------------------

    def start(self, path: Optional[Path] = None) -> Optional[Path]:
        """Begin a new recording. Returns the file path, or None on failure."""
        self.stop()
        if path is None:
            path = self._log_dir / _timestamped_name(datetime.now())
        self.path = Path(path)
        self._power.clear()
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fh = open(self.path, "a", encoding="utf-8")
            if fh.tell() == 0:
                fh.write("\t".join(COLUMNS) + "\n")
                fh.flush()
        except OSError:
            self.path = None
            return None
        self._fh = fh
        self._recording = True
        return self.path

    def stop(self) -> None:
        """End the current recording and close the file (no-op if idle)."""
        self._recording = False
        if self._fh is not None:
            try:
                self._fh.close()
            finally:
                self._fh = None

    # -- power carry-forward ------------------------------------------------

    def update_power(self, device_id: int, vbus: float, iq: float,
                     power: float) -> None:
        """Remember a motor's latest power read so status rows can include it."""
        if not self._recording:
            return
        self._power[device_id] = (float(vbus), float(iq), float(power))

    # -- sample logging -----------------------------------------------------

    def log_status(self, device_id: int, position: float, velocity: float,
                   torque: float, temperature: float,
                   faults: str = "") -> None:
        """Append one feedback sample (no-op unless recording).

        ``velocity`` is given in rad/s (the wire unit) and is recorded in RPM to
        match the GUI's fixed display units.
        """
        if not self._recording or self._fh is None:
            return
        fh = self._fh
        power = self._power.get(device_id)
        vbus_s = f"{power[0]:.3f}" if power else ""
        iq_s = f"{power[1]:.3f}" if power else ""
        power_s = f"{power[2]:.3f}" if power else ""
        row = (
            datetime.now().isoformat(timespec="milliseconds"),
            device_id,
            f"{position:.6f}",
            f"{velocity * RAD_S_TO_RPM:.6f}",
            f"{torque:.6f}",
            f"{temperature:.2f}",
            vbus_s,
            iq_s,
            power_s,
            faults,
        )
        try:
            fh.write("\t".join(str(c) for c in row) + "\n")
            fh.flush()
        except OSError:
            # Disk full / device gone: stop cleanly rather than crash the GUI.
            self.stop()

    # -- lifecycle ----------------------------------------------------------

    def close(self) -> None:
        """Alias for :meth:`stop`, called on window close."""
        self.stop()
