"""Software calibration persists per-motor and survives a reload."""

from __future__ import annotations

from robstride_gui.calibration_store import CalibrationRecord, CalibrationStore


def test_roundtrip_persists_direction_and_offset(tmp_path):
    path = tmp_path / "calibrations.json"
    store = CalibrationStore(path=path)
    store.upsert(CalibrationRecord(1, -1, 0.25))
    store.upsert(CalibrationRecord(2, 1, -0.1))
    store.save()

    loaded = CalibrationStore(path=path).load()

    assert loaded.get(1).direction == -1
    assert abs(loaded.get(1).offset - 0.25) < 1e-9
    assert loaded.get(2).direction == 1
    assert abs(loaded.get(2).offset - (-0.1)) < 1e-9
    assert loaded.get(3) is None


def test_upsert_replaces_same_device(tmp_path):
    store = CalibrationStore(path=tmp_path / "c.json")
    store.upsert(CalibrationRecord(1, 1, 0.0))
    store.upsert(CalibrationRecord(1, -1, 0.5))

    assert len(store.records) == 1
    assert store.get(1).offset == 0.5
    assert store.get(1).direction == -1


def test_load_missing_file_is_empty(tmp_path):
    store = CalibrationStore(path=tmp_path / "nope.json").load()
    assert store.records == []


def test_load_corrupt_file_is_empty(tmp_path):
    path = tmp_path / "c.json"
    path.write_text("{ not valid json")
    store = CalibrationStore(path=path).load()
    assert store.records == []
