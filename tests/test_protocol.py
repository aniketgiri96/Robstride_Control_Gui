"""Unit tests for the pure protocol + presets layers (no hardware/GUI needed)."""

from __future__ import annotations

import math
import struct

import pytest

from robstride_gui import protocol as p
from robstride_gui.presets import Preset, PresetStore
from robstride_gui.safety import Calibration


# --- per-motor calibration (invert direction + zero offset) ---------------------


def test_calibration_identity_by_default():
    c = Calibration()
    for v in (-3.0, 0.0, 1.234):
        assert c.pos_from_raw(c.pos_to_raw(v)) == pytest.approx(v)
        assert c.signed_from_raw(c.signed_to_raw(v)) == pytest.approx(v)


def test_calibration_offset_shifts_zero():
    c = Calibration(direction=1, offset=0.5)
    # commanding user 0 sends raw 0.5; raw 0.5 reads back as user 0
    assert c.pos_to_raw(0.0) == pytest.approx(0.5)
    assert c.pos_from_raw(0.5) == pytest.approx(0.0)


def test_calibration_invert_flips_direction_not_offset():
    c = Calibration(direction=-1, offset=0.0)
    assert c.pos_to_raw(1.0) == pytest.approx(-1.0)
    assert c.signed_to_raw(2.0) == pytest.approx(-2.0)
    # round-trip still exact because direction*direction == 1
    assert c.pos_from_raw(c.pos_to_raw(0.7)) == pytest.approx(0.7)


def test_calibration_two_motors_independent_directions():
    left = Calibration(direction=1)
    right = Calibration(direction=-1)
    # one user command, opposite raw motion -> mirrored multi-motor rig
    assert left.pos_to_raw(1.0) == pytest.approx(1.0)
    assert right.pos_to_raw(1.0) == pytest.approx(-1.0)


# --- extended id packing --------------------------------------------------------


def test_ext_id_roundtrip():
    for comm, extra, dev in [(3, 0xFD, 1), (18, 0x00, 127), (1, 0xFFFF, 5)]:
        ext = p.make_ext_id(comm, extra, dev)
        assert p.split_ext_id(ext) == (comm, extra, dev)


@pytest.mark.parametrize("comm,extra,dev", [(32, 0, 1), (0, 0x1FFFF, 1), (0, 0, 256)])
def test_make_ext_id_rejects_out_of_range(comm, extra, dev):
    with pytest.raises(ValueError):
        p.make_ext_id(comm, extra, dev)


# --- AT framing -----------------------------------------------------------------


def test_at_frame_matches_vendor_tail():
    """The 4-byte id field must end in 07 e8 0c for host=0xFD, motor id=1.

    This tail is taken verbatim from the working vendor motor_zero.py frames,
    e.g. '41542007e80c08...'. It pins down our (ext_id<<3)|flag serialization.
    """
    frame = p.build_enable(1)              # host 0xFD, id 1
    encoded = p.encode_at(frame)
    assert encoded[:2] == b"AT"
    assert encoded[-2:] == b"\r\n"
    id_field = encoded[2:6]
    assert id_field.hex().endswith("07e80c"), id_field.hex()
    # the full 4-byte id field must round-trip to (ENABLE, host=0xFD, id=1):
    # checking only the low bytes let a wrong comm_type (high byte) slip through.
    expected = (p.make_ext_id(p.CommunicationType.ENABLE, 0xFD, 1) << 3) | p.AT_ID_FLAG
    assert int.from_bytes(id_field, "big") == expected, id_field.hex()
    # dlc byte follows the 4-byte id, and a real frame carries 8 data bytes
    assert encoded[6] == len(frame.data) == 8

    # a ping must also carry a full 8-byte payload (adapter rejects dlc=0)
    assert p.encode_at(p.build_ping(1))[6] == 8


def test_at_encode_decode_roundtrip_all_builders():
    builders = [
        p.build_ping(2),
        p.build_enable(3),
        p.build_disable(4),
        p.build_set_zero(5),
        p.build_read_param(6, p.ParameterType.MODE),
        p.build_write_param(7, p.ParameterType.VELOCITY_TARGET, 3.5),
        p.build_operation(8, 1.57, 0.0, 28.0, 6.0, 0.0, "rs-04"),
    ]
    for original in builders:
        frames, rest = p.decode_at(p.encode_at(original))
        assert rest == b""
        assert len(frames) == 1
        decoded = frames[0]
        assert decoded.comm_type == original.comm_type
        assert decoded.extra_data == original.extra_data
        assert decoded.device_id == original.device_id
        assert decoded.data == original.data


def test_build_set_id_frame():
    frame = p.build_set_id(current_id=1, new_id=5)
    assert frame.comm_type == p.CommunicationType.SET_DEVICE_ID
    assert frame.device_id == 1          # targets the current id
    # new id rides in the UPPER byte of data area 2 (bits 23..16), host id in
    # the lower byte (bits 15..8): extra_data == (5 << 8) | 0xFD.
    assert frame.extra_data == (5 << 8) | p.DEFAULT_HOST_ID
    # the actual target-id field (bits 23..16 of the ext id) must be the new id
    comm, extra, dev = p.split_ext_id(frame.ext_id)
    assert (extra >> 8) & 0xFF == 5      # new id
    assert extra & 0xFF == p.DEFAULT_HOST_ID  # host id
    assert dev == 1                      # current id
    # round-trips through AT framing
    frames, rest = p.decode_at(p.encode_at(frame))
    assert rest == b"" and len(frames) == 1
    assert frames[0].device_id == 1
    assert frames[0].extra_data == (5 << 8) | p.DEFAULT_HOST_ID


def test_build_set_id_rejects_bad_new_id():
    with pytest.raises(ValueError):
        p.build_set_id(1, 0)
    with pytest.raises(ValueError):
        p.build_set_id(1, 200)


def test_decode_at_handles_partial_and_multiple():
    a = p.encode_at(p.build_enable(1))
    b = p.encode_at(p.build_disable(2))
    stream = a + b
    # feed split across a boundary in the middle of frame b
    cut = len(a) + 3
    frames1, rest1 = p.decode_at(stream[:cut])
    assert len(frames1) == 1                  # only frame a is complete
    frames2, rest2 = p.decode_at(rest1 + stream[cut:])
    assert len(frames2) == 1                  # frame b completes
    assert rest2 == b""


# --- MIT scaling ----------------------------------------------------------------


@pytest.mark.parametrize("angle", [-math.pi, -1.0, 0.0, 1.0, math.pi])
def test_mit_position_roundtrip_rs04(angle):
    op = p.build_operation(1, angle, 0, 0, 0, 0, "rs-04")
    pos_u16 = struct.unpack(">HHHH", op.data)[0]
    back = (pos_u16 / 0x7FFF - 1.0) * (4 * math.pi)
    assert abs(back - angle) < 0.01


def test_operation_clamps_beyond_range():
    # velocity for rs-04 caps at 15 rad/s; 100 must clamp to the max code.
    # The vendor scaling maps +vmax -> (1+1)*0x7FFF = 0xFFFE (not 0xFFFF).
    op = p.build_operation(1, 0, 100.0, 0, 0, 0, "rs-04")
    vel_u16 = struct.unpack(">HHHH", op.data)[1]
    assert vel_u16 >= 0xFFFE
    # and the negative extreme clamps to 0
    op_neg = p.build_operation(1, 0, -100.0, 0, 0, 0, "rs-04")
    assert struct.unpack(">HHHH", op_neg.data)[1] == 0x0000


def test_model_limits_unknown_falls_back_to_rs04():
    assert p.model_limits("nope") == p.model_limits("rs-04")
    assert p.model_limits("rs-04")["velocity"] == 15


# --- status / param parsing -----------------------------------------------------


def test_parse_status_decodes_fields_and_faults():
    data = struct.pack(">HHHH", 0x7FFF, 0x7FFF, 0x7FFF, 305)  # ~0, 30.5 C
    extra = (1 << 12) | 0x01  # stall bit set, device id 1
    frame = p.Frame(p.CommunicationType.OPERATION_STATUS, extra, 0, data)
    st = p.parse_status(frame, "rs-04")
    assert st.device_id == 1
    assert abs(st.position) < 0.01
    assert abs(st.temperature - 30.5) < 0.01
    assert st.stalled and st.has_fault


def test_parse_param_value_float():
    raw = struct.pack("<f", 2.75)
    frame = p.Frame(p.CommunicationType.READ_PARAMETER, 0x01, 0,
                    struct.pack("<HH", 0x700A, 0) + raw)
    assert abs(p.parse_param_value(frame, p.ParameterType.VELOCITY_TARGET) - 2.75) < 1e-6


# --- presets --------------------------------------------------------------------


def test_preset_store_roundtrip(tmp_path):
    path = tmp_path / "presets.json"
    store = PresetStore(path=path)
    store.upsert(Preset(name="home", device_id=1, mode=1, position=0.0))
    store.upsert(Preset(name="reach", device_id=1, mode=1, position=math.pi))
    store.save()

    reloaded = PresetStore(path=path).load()
    assert reloaded.names() == ["home", "reach"]
    assert abs(reloaded.get("reach").position - math.pi) < 1e-9

    # upsert replaces by name
    reloaded.upsert(Preset(name="home", device_id=2, mode=2, velocity=5.0))
    assert reloaded.get("home").device_id == 2
    assert reloaded.remove("home") is True
    assert reloaded.names() == ["reach"]


def test_preset_store_missing_file_is_empty(tmp_path):
    store = PresetStore(path=tmp_path / "nope.json").load()
    assert store.names() == []
