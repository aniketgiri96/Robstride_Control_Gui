"""Per-motor software calibration persisted to a JSON file.

The GUI-side zero/direction trim (:class:`robstride_gui.safety.Calibration`)
otherwise lives only in the worker's memory and is lost when the GUI closes.
This store mirrors it to disk, keyed by CAN id, so a captured software zero
survives a restart - the counterpart to saving the *hardware* zero to flash.

On-disk schema (``~/.config/robstride_gui/calibrations.json``)::

    {
      "version": 1,
      "calibrations": [
        {"device_id": 1, "direction": 1, "offset": 0.0}
      ]
    }
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path

SCHEMA_VERSION = 1


def default_calibrations_path() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME") or os.path.join(Path.home(), ".config")
    return Path(base) / "robstride_gui" / "calibrations.json"


@dataclass
class CalibrationRecord:
    device_id: int
    direction: int = 1     # +1 normal, -1 inverted
    offset: float = 0.0    # rad, raw frame
    # Calibrated travel range in the *user* frame (rad). ``None`` means that
    # bound was never calibrated and no software limit is enforced on that side.
    # Old files that predate these fields load fine: ``from_dict`` drops unknown
    # keys and these defaults fill the missing ones.
    pos_min: float | None = None
    pos_max: float | None = None
    # Dashboard calibration lock: when True the overview screen's CV1/CV2 edits
    # are disabled so they cannot be nudged during operation. Defaults to locked
    # (the safe state) and, like the range fields, is backfilled for old files.
    locked: bool = True

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "CalibrationRecord":
        known = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]
        return cls(**{k: v for k, v in d.items() if k in known})


@dataclass
class CalibrationStore:
    """In-memory collection of per-motor calibrations with load/save to ``path``."""

    path: Path = field(default_factory=default_calibrations_path)
    records: list[CalibrationRecord] = field(default_factory=list)

    def load(self) -> "CalibrationStore":
        try:
            raw = json.loads(Path(self.path).read_text())
        except FileNotFoundError:
            self.records = []
            return self
        except (json.JSONDecodeError, OSError):
            # Corrupt or unreadable file: start clean rather than crash the GUI.
            self.records = []
            return self
        items = raw.get("calibrations", []) if isinstance(raw, dict) else []
        self.records = [CalibrationRecord.from_dict(d) for d in items
                        if isinstance(d, dict) and "device_id" in d]
        return self

    def save(self) -> None:
        path = Path(self.path)
        path.parent.mkdir(parents=True, exist_ok=True)
        doc = {"version": SCHEMA_VERSION,
               "calibrations": [r.to_dict() for r in self.records]}
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(doc, indent=2))
        tmp.replace(path)  # atomic on POSIX

    def upsert(self, record: CalibrationRecord) -> None:
        """Add ``record`` or replace an existing one with the same device_id."""
        for i, existing in enumerate(self.records):
            if existing.device_id == record.device_id:
                self.records[i] = record
                break
        else:
            self.records.append(record)

    def get(self, device_id: int) -> CalibrationRecord | None:
        return next((r for r in self.records if r.device_id == device_id), None)
