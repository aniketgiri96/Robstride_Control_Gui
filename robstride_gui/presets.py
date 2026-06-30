"""Motion presets: named target setpoints persisted to a JSON file.

A preset captures everything needed to re-issue a command to a motor: which
motor, which run-mode, and the target values + gains. Presets are stored in a
single JSON document under the user's config dir so they survive restarts.

On-disk schema (``~/.config/robstride_gui/presets.json``)::

    {
      "version": 1,
      "presets": [
        {"name": "home", "device_id": 1, "mode": 1,
         "position": 0.0, "velocity": 0.0, "current": 0.0,
         "kp": 28.0, "kd": 6.0}
      ]
    }
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path

SCHEMA_VERSION = 1


def default_presets_path() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME") or os.path.join(Path.home(), ".config")
    return Path(base) / "robstride_gui" / "presets.json"


@dataclass
class Preset:
    name: str
    device_id: int
    mode: int = 1            # RunMode value
    position: float = 0.0    # rad
    velocity: float = 0.0    # rad/s
    current: float = 0.0     # A
    kp: float = 28.0
    kd: float = 6.0

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Preset":
        known = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]
        return cls(**{k: v for k, v in d.items() if k in known})


@dataclass
class PresetStore:
    """In-memory collection of presets with load/save to ``path``."""

    path: Path = field(default_factory=default_presets_path)
    presets: list[Preset] = field(default_factory=list)

    def load(self) -> "PresetStore":
        try:
            raw = json.loads(Path(self.path).read_text())
        except FileNotFoundError:
            self.presets = []
            return self
        except (json.JSONDecodeError, OSError):
            # Corrupt or unreadable file: start clean rather than crash the GUI.
            self.presets = []
            return self
        items = raw.get("presets", []) if isinstance(raw, dict) else []
        self.presets = [Preset.from_dict(d) for d in items if isinstance(d, dict)]
        return self

    def save(self) -> None:
        path = Path(self.path)
        path.parent.mkdir(parents=True, exist_ok=True)
        doc = {"version": SCHEMA_VERSION,
               "presets": [p.to_dict() for p in self.presets]}
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(doc, indent=2))
        tmp.replace(path)  # atomic on POSIX

    def upsert(self, preset: Preset) -> None:
        """Add ``preset`` or replace an existing one with the same name."""
        for i, existing in enumerate(self.presets):
            if existing.name == preset.name:
                self.presets[i] = preset
                break
        else:
            self.presets.append(preset)

    def remove(self, name: str) -> bool:
        before = len(self.presets)
        self.presets = [p for p in self.presets if p.name != name]
        return len(self.presets) != before

    def get(self, name: str) -> Preset | None:
        return next((p for p in self.presets if p.name == name), None)

    def names(self) -> list[str]:
        return [p.name for p in self.presets]
