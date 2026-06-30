"""RobStride CAN protocol: constants, register map, and frame (de)serialization.

This module is **pure** (only ``struct``/``math`` from the stdlib) so it can be
unit-tested without any hardware or GUI dependencies.

Wire model
----------
Every RobStride command is a CAN 2.0 **extended** frame whose 29-bit
arbitration ID packs three fields::

    ext_id = (comm_type << 24) | (extra_data << 8) | device_id
              \\__ 5 bits __/    \\___ 16 bits ___/   \\_ 8 bits _/

* ``comm_type``  - what the frame does (see :class:`CommunicationType`).
* ``extra_data`` - "data area 2": the host/master CAN id for outgoing frames,
  or the packed motor status word for incoming feedback frames.
* ``device_id``  - target motor CAN id (1..255).

The data payload is up to 8 bytes, **big-endian** for MIT operation frames and
feedback, little-endian for parameter read/write.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Final

# --- adapter / framing constants -------------------------------------------------

#: Default host (master) CAN id placed in ``extra_data`` of outgoing frames.
#: 0xFD matches the value captured from the working ``motor_zero.py`` AT frames.
DEFAULT_HOST_ID: Final = 0xFD

#: Default serial baud rate for the USB-CAN adapter in AT mode.
DEFAULT_SERIAL_BAUD: Final = 921600

#: Default SocketCAN bitrate.
DEFAULT_CAN_BITRATE: Final = 1_000_000

#: Low-order flag nibble the AT adapter expects in its 4-byte id field
#: (``(ext_id << 3) | AT_ID_FLAG``). 0x04 marks an extended frame.
AT_ID_FLAG: Final = 0x04


class CommunicationType:
    """RobStride ``comm_type`` values (upper 5 bits of the extended id)."""

    GET_DEVICE_ID = 0
    OPERATION_CONTROL = 1   # MIT frame: target pos/vel/kp/kd + torque ff
    OPERATION_STATUS = 2    # motor feedback: pos/vel/torque/temperature
    ENABLE = 3
    DISABLE = 4
    SET_ZERO_POSITION = 6
    SET_DEVICE_ID = 7
    READ_PARAMETER = 17
    WRITE_PARAMETER = 18
    FAULT_REPORT = 21
    SAVE_PARAMETERS = 22
    SET_BAUDRATE = 23
    ACTIVE_REPORT = 24
    SET_PROTOCOL = 25


class RunMode:
    """Values for the ``run_mode`` register (0x7005)."""

    MIT = 0          # direct operation / impedance control
    POSITION_PP = 1  # profiled position
    VELOCITY = 2
    CURRENT = 3
    POSITION_CSP = 5  # cyclic-sync position

    NAMES: Final = {
        MIT: "MIT",
        POSITION_PP: "Position (PP)",
        VELOCITY: "Velocity",
        CURRENT: "Current",
        POSITION_CSP: "Position (CSP)",
    }


# --- register / parameter map ----------------------------------------------------


@dataclass(frozen=True)
class Param:
    """A single readable/writable motor parameter."""

    index: int
    fmt: str   # struct format char for the value: 'f','b','B','h','H','l','L'
    name: str


class ParameterType:
    """Register addresses (index, value struct-format, human name)."""

    MECHANICAL_OFFSET = Param(0x2005, "f", "mechOffset")
    MEASURED_POSITION = Param(0x3016, "f", "mechPos")
    MEASURED_VELOCITY = Param(0x3017, "f", "mechVel")
    MEASURED_TORQUE = Param(0x302C, "f", "torque_fdb")
    MODE = Param(0x7005, "b", "run_mode")
    IQ_TARGET = Param(0x7006, "f", "iq_ref")
    TORQUE_TARGET = Param(0x7007, "f", "torque_ref")
    VELOCITY_TARGET = Param(0x700A, "f", "spd_ref")
    TORQUE_LIMIT = Param(0x700B, "f", "limit_torque")
    CURRENT_KP = Param(0x7010, "f", "cur_kp")
    CURRENT_KI = Param(0x7011, "f", "cur_ki")
    CURRENT_FILTER_GAIN = Param(0x7014, "f", "cur_filter_gain")
    POSITION_TARGET = Param(0x7016, "f", "loc_ref")
    VELOCITY_LIMIT = Param(0x7017, "f", "limit_spd")
    CURRENT_LIMIT = Param(0x7018, "f", "limit_cur")
    MECHANICAL_POSITION = Param(0x7019, "f", "mechPos")
    IQ_FILTERED = Param(0x701A, "f", "iqf")
    MECHANICAL_VELOCITY = Param(0x701B, "f", "mechVel")
    VBUS = Param(0x701C, "f", "VBUS")
    POSITION_KP = Param(0x701E, "f", "loc_kp")
    VELOCITY_KP = Param(0x701F, "f", "spd_kp")
    VELOCITY_KI = Param(0x7020, "f", "spd_ki")
    VELOCITY_FILTER_GAIN = Param(0x7021, "f", "spd_filter_gain")
    VEL_ACCELERATION_TARGET = Param(0x7022, "f", "acc_rad")
    PP_VELOCITY_MAX = Param(0x7024, "f", "vel_max")
    PP_ACCELERATION_TARGET = Param(0x7025, "f", "acc_set")
    CAN_TIMEOUT = Param(0x7028, "L", "canTimeout")
    ZERO_STATE = Param(0x7029, "B", "zero_sta")


# --- per-model MIT scaling tables (datasheet ranges) ----------------------------

_4PI = 4 * math.pi

MODEL_POSITION_MAX: Final = {m: _4PI for m in
                             ("rs-00", "rs-01", "rs-02", "rs-03", "rs-04", "rs-05", "rs-06")}
MODEL_VELOCITY_MAX: Final = {"rs-00": 50, "rs-01": 44, "rs-02": 44, "rs-03": 50,
                             "rs-04": 15, "rs-05": 33, "rs-06": 20}
MODEL_TORQUE_MAX: Final = {"rs-00": 14, "rs-01": 17, "rs-02": 17, "rs-03": 60,
                           "rs-04": 120, "rs-05": 17, "rs-06": 36}
MODEL_KP_MAX: Final = {"rs-00": 500.0, "rs-01": 500.0, "rs-02": 500.0, "rs-03": 5000.0,
                       "rs-04": 5000.0, "rs-05": 500.0, "rs-06": 5000.0}
MODEL_KD_MAX: Final = {"rs-00": 5.0, "rs-01": 5.0, "rs-02": 5.0, "rs-03": 100.0,
                       "rs-04": 100.0, "rs-05": 5.0, "rs-06": 100.0}

DEFAULT_MODEL: Final = "rs-04"
MODELS: Final = tuple(MODEL_VELOCITY_MAX.keys())


def model_limits(model: str) -> dict[str, float]:
    """Return the MIT scaling limits for ``model`` (falls back to rs-04)."""
    if model not in MODEL_VELOCITY_MAX:
        model = DEFAULT_MODEL
    return {
        "position": float(MODEL_POSITION_MAX[model]),
        "velocity": float(MODEL_VELOCITY_MAX[model]),
        "torque": float(MODEL_TORQUE_MAX[model]),
        "kp": float(MODEL_KP_MAX[model]),
        "kd": float(MODEL_KD_MAX[model]),
    }


# --- small helpers ---------------------------------------------------------------


def _clip(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def _to_u16(value: float, vmax: float, signed_range: bool = True) -> int:
    """Map a physical value into a 0..0xFFFF code.

    ``signed_range=True`` maps ``[-vmax, vmax] -> [0, 0xFFFF]`` (used for
    position/velocity/torque). ``False`` maps ``[0, vmax] -> [0, 0xFFFF]``
    (used for kp/kd).
    """
    if signed_range:
        value = _clip(value, -vmax, vmax)
        code = int((value / vmax + 1.0) * 0x7FFF)
    else:
        value = _clip(value, 0.0, vmax)
        code = int((value / vmax) * 0xFFFF)
    return int(_clip(code, 0, 0xFFFF))


def _from_u16(code: int, vmax: float) -> float:
    """Inverse of the signed :func:`_to_u16` mapping."""
    return (float(code) / 0x7FFF - 1.0) * vmax


# --- extended-id <-> field helpers ----------------------------------------------


def make_ext_id(comm_type: int, extra_data: int, device_id: int) -> int:
    """Pack the three fields into a 29-bit extended arbitration id."""
    if not 0 <= comm_type <= 0x1F:
        raise ValueError(f"comm_type out of range: {comm_type}")
    if not 0 <= extra_data <= 0xFFFF:
        raise ValueError(f"extra_data out of range: {extra_data}")
    if not 0 <= device_id <= 0xFF:
        raise ValueError(f"device_id out of range: {device_id}")
    return (comm_type << 24) | (extra_data << 8) | device_id


def split_ext_id(ext_id: int) -> tuple[int, int, int]:
    """Inverse of :func:`make_ext_id` -> ``(comm_type, extra_data, device_id)``."""
    comm_type = (ext_id >> 24) & 0x1F
    extra_data = (ext_id >> 8) & 0xFFFF
    device_id = ext_id & 0xFF
    return comm_type, extra_data, device_id


@dataclass(frozen=True)
class Frame:
    """A protocol-level CAN frame, independent of the wire transport."""

    comm_type: int
    extra_data: int
    device_id: int
    data: bytes = b""

    @property
    def ext_id(self) -> int:
        return make_ext_id(self.comm_type, self.extra_data, self.device_id)


# --- AT serial framing -----------------------------------------------------------

AT_HEAD: Final = b"AT"
AT_TAIL: Final = b"\r\n"


def encode_at(frame: Frame) -> bytes:
    """Serialize a :class:`Frame` for a USB-CAN adapter in AT mode.

    Layout: ``b"AT" + uint32_be((ext_id << 3) | AT_ID_FLAG) + dlc + data + b"\\r\\n"``.
    Verified against the known-good frames in the vendor ``motor_zero.py``.
    """
    if len(frame.data) > 8:
        raise ValueError("data length exceeds 8 bytes")
    id_field = (frame.ext_id << 3) | AT_ID_FLAG
    return AT_HEAD + struct.pack(">I", id_field) + bytes([len(frame.data)]) + frame.data + AT_TAIL


def decode_at(buffer: bytes) -> tuple[list[Frame], bytes]:
    """Parse zero or more AT frames out of ``buffer``.

    Returns ``(frames, remainder)`` where ``remainder`` holds an incomplete
    trailing fragment to be prepended to the next read.
    """
    frames: list[Frame] = []
    i = 0
    n = len(buffer)
    while True:
        start = buffer.find(AT_HEAD, i)
        if start < 0:
            # keep at most the last byte in case it is a stray 'A' of 'AT'
            return frames, buffer[max(i, n - 1):]
        # need head(2) + id(4) + dlc(1)
        if start + 7 > n:
            return frames, buffer[start:]
        id_field = struct.unpack(">I", buffer[start + 2:start + 6])[0]
        dlc = buffer[start + 6]
        end = start + 7 + dlc + len(AT_TAIL)
        if dlc > 8:
            # malformed dlc; skip this head and resync
            i = start + 2
            continue
        if end > n:
            # truncated frame: wait for more bytes
            return frames, buffer[start:]
        data = buffer[start + 7:start + 7 + dlc]
        ext_id = id_field >> 3
        comm_type, extra_data, device_id = split_ext_id(ext_id)
        frames.append(Frame(comm_type, extra_data, device_id, data))
        i = end


# --- command builders ------------------------------------------------------------


def build_enable(device_id: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    return Frame(CommunicationType.ENABLE, host_id, device_id, b"\x00" * 8)


def build_disable(device_id: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    return Frame(CommunicationType.DISABLE, host_id, device_id, b"\x00" * 8)


def build_set_zero(device_id: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    return Frame(CommunicationType.SET_ZERO_POSITION, host_id, device_id, b"\x01" + b"\x00" * 7)


def build_ping(device_id: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    # Must carry a full 8-byte payload: the USB-CAN adapter rejects a dlc=0
    # frame (returns a short error status instead of forwarding it), and every
    # vendor-verified frame uses dlc=8.
    return Frame(CommunicationType.GET_DEVICE_ID, host_id, device_id, b"\x00" * 8)


def build_set_id(current_id: int, new_id: int,
                 host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Reassign a motor's CAN id (comm type 7).

    The frame targets the motor's *current* id. Per the RobStride/CyberGear
    spec the new id rides in the **upper byte** of data area 2 (bits 23..16 of
    the extended id) and the host id in the lower byte (bits 15..8), so
    ``extra_data = (new_id << 8) | host_id``. Putting the new id in the low byte
    instead leaves the real target-id field zero and the motor never adopts it -
    the change silently no-ops. Only one motor should be on the bus when this is
    sent.
    """
    if not 1 <= new_id <= 0x7F:
        raise ValueError(f"new_id out of range (1..127): {new_id}")
    extra_data = ((new_id & 0xFF) << 8) | (host_id & 0xFF)
    return Frame(CommunicationType.SET_DEVICE_ID, extra_data, current_id,
                 b"\x00" * 8)


def build_save(device_id: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Persist the motor's parameters to flash (comm type 22).

    Sent after a SET_DEVICE_ID so the reassigned CAN id survives a power cycle;
    without it the change lives only in RAM and is lost on the next reboot.
    """
    return Frame(CommunicationType.SAVE_PARAMETERS, host_id, device_id, b"\x00" * 8)


def build_read_param(device_id: int, param: Param, host_id: int = DEFAULT_HOST_ID) -> Frame:
    data = struct.pack("<HHL", param.index, 0x00, 0x00)
    return Frame(CommunicationType.READ_PARAMETER, host_id, device_id, data)


def build_write_param(device_id: int, param: Param, value: float | int,
                      host_id: int = DEFAULT_HOST_ID) -> Frame:
    value_buf = struct.pack("<" + param.fmt, _coerce(param.fmt, value))
    value_buf = value_buf + b"\x00" * (4 - len(value_buf))  # pad to 4 data bytes
    data = struct.pack("<HH", param.index, 0x00) + value_buf
    return Frame(CommunicationType.WRITE_PARAMETER, host_id, device_id, data)


def _coerce(fmt: str, value: float | int) -> float | int:
    return float(value) if fmt == "f" else int(value)


def build_operation(device_id: int, position: float, velocity: float,
                    kp: float, kd: float, torque_ff: float, model: str) -> Frame:
    """Build an MIT operation-control frame.

    ``extra_data`` carries the torque feedforward code; the data payload packs
    position/velocity/kp/kd as big-endian u16 codes.
    """
    lim = model_limits(model)
    pos_u16 = _to_u16(position, lim["position"], signed_range=True)
    vel_u16 = _to_u16(velocity, lim["velocity"], signed_range=True)
    kp_u16 = _to_u16(kp, lim["kp"], signed_range=False)
    kd_u16 = _to_u16(kd, lim["kd"], signed_range=False)
    tq_u16 = _to_u16(torque_ff, lim["torque"], signed_range=True)
    data = struct.pack(">HHHH", pos_u16, vel_u16, kp_u16, kd_u16)
    return Frame(CommunicationType.OPERATION_CONTROL, tq_u16, device_id, data)


# --- feedback parsing ------------------------------------------------------------


@dataclass(frozen=True)
class MotorStatus:
    """Decoded feedback from an :data:`CommunicationType.OPERATION_STATUS` frame."""

    device_id: int
    position: float       # rad
    velocity: float       # rad/s
    torque: float         # Nm
    temperature: float    # deg C
    uncalibrated: bool = False
    stalled: bool = False
    encoder_fault: bool = False
    overtemperature: bool = False
    overcurrent: bool = False
    undervoltage: bool = False

    @property
    def has_fault(self) -> bool:
        return any((self.stalled, self.encoder_fault, self.overtemperature,
                    self.overcurrent, self.undervoltage))


def parse_status(frame: Frame, model: str) -> MotorStatus:
    """Decode an OPERATION_STATUS feedback frame into a :class:`MotorStatus`."""
    extra = frame.extra_data
    device_id = extra & 0xFF
    lim = model_limits(model)
    pos_u16, vel_u16, tq_i16, temp_u16 = struct.unpack(">HHHH", frame.data[:8])
    return MotorStatus(
        device_id=device_id,
        position=_from_u16(pos_u16, lim["position"]),
        velocity=_from_u16(vel_u16, lim["velocity"]),
        torque=_from_u16(tq_i16, lim["torque"]),
        temperature=float(temp_u16) * 0.1,
        uncalibrated=bool((extra >> 13) & 0x01),
        stalled=bool((extra >> 12) & 0x01),
        encoder_fault=bool((extra >> 11) & 0x01),
        overtemperature=bool((extra >> 10) & 0x01),
        overcurrent=bool((extra >> 9) & 0x01),
        undervoltage=bool((extra >> 8) & 0x01),
    )


def parse_param_value(frame: Frame, param: Param) -> float | int:
    """Decode a READ_PARAMETER response payload for ``param``."""
    payload = frame.data[4:]
    raw = payload[:struct.calcsize("<" + param.fmt)]
    return struct.unpack("<" + param.fmt, raw)[0]
