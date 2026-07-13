"""High-level RobStride motor bus built on top of a :class:`Transport`.

Owns a transport plus a registry of motors (by CAN id + model) and exposes the
operations a GUI needs: ping/scan, enable/disable, set-zero, parameter read &
write, run-mode switching, MIT operation frames, and feedback decoding.

All methods are synchronous and meant to be driven from a single worker thread
(see :mod:`robstride_gui.worker`); the class is **not** internally locked.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional

from . import protocol as proto
from .protocol import (
    CommunicationType,
    Frame,
    MotorStatus,
    Param,
    ParameterType,
)
from .transport import Transport

logger = logging.getLogger(__name__)


#: Reverse map comm_type value -> name for readable frame logs. Built from the
#: int constants on CommunicationType (a plain constants class, not an IntEnum).
_COMM_TYPE_NAMES: dict[int, str] = {
    value: name
    for name, value in vars(CommunicationType).items()
    if not name.startswith("_") and isinstance(value, int)
}


def _frame_desc(frame: Frame) -> str:
    """One-line human description of a CAN frame for TX/RX debug logging.

    Names the comm-type (ENABLE/DISABLE/WRITE_PARAMETER/OPERATION_STATUS/...),
    the id byte, the host/extra field, and the raw payload. The comm-type + id
    is what makes a cross-motor bug visible on the wire: e.g. a DISABLE carrying
    id=6 appearing while only motor 4 was commanded, or a motor self-emitting an
    OPERATION_STATUS while we addressed a different one. Both ``device_id`` and
    ``extra`` are shown because a reply can carry the motor id in either field
    (which is why the match-filter checks both)."""
    name = _COMM_TYPE_NAMES.get(frame.comm_type, f"type{frame.comm_type}")
    return (f"{name} id={frame.device_id} extra=0x{frame.extra_data:04X} "
            f"data={frame.data.hex()}")


@dataclass
class Motor:
    """A motor registered on the bus."""

    device_id: int
    model: str = proto.DEFAULT_MODEL
    name: str = ""

    def __post_init__(self):
        if not self.name:
            self.name = f"M{self.device_id}"


@dataclass
class BusConfig:
    host_id: int = proto.DEFAULT_HOST_ID
    response_timeout: float = 0.15
    inter_command_delay: float = 0.0


class RobstrideBus:
    """Coordinates a transport and a set of motors."""

    def __init__(self, transport: Transport, config: Optional[BusConfig] = None):
        self.transport = transport
        self.config = config or BusConfig()
        self.motors: dict[int, Motor] = {}

    # -- lifecycle ---------------------------------------------------------------

    def open(self) -> None:
        self.transport.open()

    def close(self) -> None:
        self.transport.close()

    @property
    def is_open(self) -> bool:
        return self.transport.is_open

    def add_motor(self, motor: Motor) -> None:
        self.motors[motor.device_id] = motor

    def model_of(self, device_id: int) -> str:
        m = self.motors.get(device_id)
        return m.model if m else proto.DEFAULT_MODEL

    # -- low-level request/response ---------------------------------------------

    def _send(self, frame: Frame) -> None:
        self._log_frame("TX", frame)
        self.transport.send(frame)
        if self.config.inter_command_delay:
            time.sleep(self.config.inter_command_delay)

    def _log_frame(self, direction: str, frame: Frame) -> None:
        """Log one frame at DEBUG (enable with ROBSTRIDE_DEBUG=1). No-op and
        near-zero cost otherwise. See :func:`_frame_desc` for the format."""
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("%s %s", direction, _frame_desc(frame))

    def _recv_logged(self, timeout: float) -> Optional[Frame]:
        """recv one frame and log EVERY frame *before* any match-filtering, so an
        unsolicited frame - e.g. a motor self-reporting DISABLED while we were
        addressing a different motor - is visible instead of silently dropped by
        the caller's comm-type/id filter. This is the point of the capture."""
        frame = self.transport.recv(timeout=timeout)
        if frame is not None:
            self._log_frame("RX", frame)
        return frame

    def _await(self, comm_types: tuple[int, ...], device_id: Optional[int],
               timeout: Optional[float] = None) -> Optional[Frame]:
        """Wait for the next frame matching ``comm_types`` (and id, if given)."""
        timeout = self.config.response_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self._recv_logged(max(0.0, deadline - time.monotonic()))
            if frame is None:
                continue
            if frame.comm_type not in comm_types:
                continue
            if device_id is not None and (frame.extra_data & 0xFF) != device_id \
                    and frame.device_id != device_id:
                continue
            return frame
        return None

    # -- discovery ---------------------------------------------------------------

    def ping(self, device_id: int, timeout: float = 0.1) -> bool:
        self._send(proto.build_ping(device_id, self.config.host_id))
        return self._await((CommunicationType.GET_DEVICE_ID,
                            CommunicationType.OPERATION_STATUS), device_id, timeout) is not None

    def scan(self, start: int = 1, end: int = 16, timeout: float = 0.02) -> list[int]:
        # A short per-id timeout keeps a full 0..127 sweep quick (~2s instead of
        # ~8s). A present motor replies in a few ms and ping() returns on the
        # first answer, so only empty ids ever wait the whole window; this is
        # what stops a Detect from freezing the single worker thread's live
        # feedback for several seconds.
        found = []
        for device_id in range(start, end + 1):
            if self.ping(device_id, timeout):
                found.append(device_id)
        return found

    def identify(self, device_id: int, timeout: float = 0.15) -> list[bytes]:
        """List the unique MCU ids of every motor answering a ping to ``device_id``.

        Each RobStride motor replies to a ping with its permanent 64-bit MCU id
        in the payload, so the returned (de-duplicated, arrival-ordered) list has
        one entry per *physical* motor on this CAN id. More than one entry means
        several motors share the id - a bus-id collision: every command to it
        drives them all in lock-step. Unlike :meth:`ping`, this listens for the
        *whole* window instead of returning on the first reply, so it can see a
        second responder.
        """
        self._send(proto.build_ping(device_id, self.config.host_id))
        deadline = time.monotonic() + timeout
        seen: list[bytes] = []
        while time.monotonic() < deadline:
            frame = self._recv_logged(max(0.0, deadline - time.monotonic()))
            if frame is None:
                continue
            if frame.comm_type not in (CommunicationType.GET_DEVICE_ID,
                                       CommunicationType.OPERATION_STATUS):
                continue
            if (frame.extra_data & 0xFF) != device_id and frame.device_id != device_id:
                continue
            if frame.data not in seen:
                seen.append(frame.data)
        return seen

    def count_responders(self, device_id: int, timeout: float = 0.15) -> int:
        """How many distinct motors answer a ping to ``device_id`` (see :meth:`identify`)."""
        return len(self.identify(device_id, timeout))

    def find_collisions(self, device_ids: list[int]) -> list[int]:
        """Return the subset of ``device_ids`` that more than one motor answers."""
        return [did for did in device_ids if self.count_responders(did) > 1]

    def inventory(self, start: int = 1, end: int = 16,
                  identify_timeout: float = 0.15) -> list[tuple[int, list[bytes]]]:
        """Scan ``start..end`` and, for each responding CAN id, list its motors' MCU ids.

        Returns ``[(can_id, [mcu_id, ...]), ...]``. An entry whose list has more
        than one id is a collision (several motors on that CAN id).
        """
        return [(device_id, self.identify(device_id, identify_timeout))
                for device_id in self.scan(start, end)]

    # -- motor actions -----------------------------------------------------------

    def enable(self, device_id: int) -> Optional[MotorStatus]:
        self._send(proto.build_enable(device_id, self.config.host_id))
        return self._read_status(device_id)

    def disable(self, device_id: int) -> Optional[MotorStatus]:
        self._send(proto.build_disable(device_id, self.config.host_id))
        return self._read_status(device_id)

    def set_zero(self, device_id: int) -> None:
        """Set the motor's mechanical zero and persist it to flash.

        SET_ZERO_POSITION (comm 6) only rewrites the motor's *live* (RAM) zero;
        without a following SAVE_PARAMETERS (comm 22) the new zero is lost on the
        next power cycle, so the motor "forgets" its absolute position. Saving
        here - the same pattern as :meth:`set_motor_id` - makes the zero survive
        a reboot.
        """
        self._send(proto.build_set_zero(device_id, self.config.host_id))
        time.sleep(0.1)
        self._send(proto.build_save(device_id, self.config.host_id))
        time.sleep(0.15)

    def read_zero_state(self, device_id: int) -> Optional[dict]:
        """Read back the motor's own persisted zero markers.

        Returns ``{"zero_sta": int|None, "mech_offset": float|None}`` - the
        ``zero_sta`` flag (0x7029) and the stored mechanical zero offset
        (``mechOffset``, 0x2005) the motor holds in its own registers. This is
        how the GUI can *show* whether the motor remembers its absolute zero.
        Returns ``None`` if the motor answers neither read (e.g. powered off).
        """
        zero_sta = self.read_param(device_id, ParameterType.ZERO_STATE)
        mech_offset = self.read_param(device_id, ParameterType.MECHANICAL_OFFSET)
        if zero_sta is None and mech_offset is None:
            return None
        return {
            "zero_sta": None if zero_sta is None else int(zero_sta),
            "mech_offset": None if mech_offset is None else float(mech_offset),
        }

    def set_motor_id(self, current_id: int, new_id: int) -> Optional[int]:
        """Reassign a motor's CAN id, persist it, and report where it answers.

        Sends SET_DEVICE_ID, then pings to learn the id the motor *actually*
        responds at - the new id if the change took effect, else the old id
        (some firmware only adopts a new id after a power cycle). It saves to
        flash at that live id so the change survives a reboot, re-keys the local
        registry, and returns the live id (or ``None`` if the motor answers at
        neither, e.g. it was powered off). Callers must follow the motor to the
        returned id instead of assuming ``new_id`` went live.

        Connect only one motor for this; broadcasting to a bus of identical-id
        motors would set them all at once.
        """
        self._send(proto.build_set_id(current_id, new_id, self.config.host_id))
        time.sleep(0.15)
        if self.ping(new_id):
            live_id: Optional[int] = new_id
        elif self.ping(current_id):
            live_id = current_id
        else:
            live_id = None
        if live_id is not None:
            self._send(proto.build_save(live_id, self.config.host_id))
            time.sleep(0.15)
        motor = self.motors.pop(current_id, None)
        model = motor.model if motor else proto.DEFAULT_MODEL
        self.add_motor(Motor(device_id=live_id if live_id is not None else current_id,
                             model=model))
        return live_id

    def _read_status(self, device_id: int) -> Optional[MotorStatus]:
        frame = self._await((CommunicationType.OPERATION_STATUS,
                             CommunicationType.FAULT_REPORT), device_id)
        if frame is None or frame.comm_type == CommunicationType.FAULT_REPORT:
            return None
        return proto.parse_status(frame, self.model_of(device_id))

    # -- parameters --------------------------------------------------------------

    def read_param(self, device_id: int, param: Param) -> Optional[float | int]:
        self._send(proto.build_read_param(device_id, param, self.config.host_id))
        frame = self._await((CommunicationType.READ_PARAMETER,), device_id)
        if frame is None:
            return None
        try:
            return proto.parse_param_value(frame, param)
        except Exception:
            return None

    def write_param(self, device_id: int, param: Param,
                    value: float | int) -> Optional[MotorStatus]:
        """Write a parameter and return the status frame the motor acks with.

        Every WRITE_PARAMETER triggers an OPERATION_STATUS reply; returning it
        lets callers use a setpoint write as live feedback, so the control loop
        never has to send a separate (motion-disturbing) operation frame just to
        read state. Returns ``None`` if the motor does not reply in time.
        """
        self._send(proto.build_write_param(device_id, param, value, self.config.host_id))
        frame = self._await((CommunicationType.OPERATION_STATUS,), device_id)
        if frame is None:
            return None
        try:
            return proto.parse_status(frame, self.model_of(device_id))
        except Exception:
            return None

    def set_run_mode(self, device_id: int, mode: int) -> None:
        self.write_param(device_id, ParameterType.MODE, int(mode))

    # -- control -----------------------------------------------------------------

    def operation(self, device_id: int, position: float, velocity: float,
                  kp: float, kd: float, torque_ff: float = 0.0) -> Optional[MotorStatus]:
        """Send an MIT operation frame and return the motor's status reply."""
        model = self.model_of(device_id)
        self._send(proto.build_operation(device_id, position, velocity, kp, kd,
                                         torque_ff, model))
        return self._read_status(device_id)

    def set_position(self, device_id: int, position_rad: float,
                     velocity_limit: Optional[float] = None) -> Optional[MotorStatus]:
        """Position mode (run_mode 1): write loc_ref (and optional speed limit).

        Returns the status the motor acks the loc_ref write with, for feedback.
        """
        if velocity_limit is not None:
            self.write_param(device_id, ParameterType.VELOCITY_LIMIT, float(velocity_limit))
        return self.write_param(device_id, ParameterType.POSITION_TARGET, float(position_rad))

    def set_velocity(self, device_id: int, velocity_rad_s: float,
                     current_limit: Optional[float] = None) -> Optional[MotorStatus]:
        """Velocity mode (run_mode 2): write spd_ref (and optional current limit).

        Returns the status the motor acks the spd_ref write with, for feedback.
        """
        if current_limit is not None:
            self.write_param(device_id, ParameterType.CURRENT_LIMIT, float(current_limit))
        return self.write_param(device_id, ParameterType.VELOCITY_TARGET, float(velocity_rad_s))

    def set_current(self, device_id: int, current_a: float) -> Optional[MotorStatus]:
        """Current mode (run_mode 3): write iq_ref. Returns the ack status."""
        return self.write_param(device_id, ParameterType.IQ_TARGET, float(current_a))

    def poll_status(self, device_id: int) -> Optional[MotorStatus]:
        """Elicit a fresh feedback frame with a zero-stiffness MIT frame.

        Sending kp=kd=0 commands no motion but the motor still replies with a
        status frame, which we decode for live position/velocity/torque.
        """
        frame = proto.build_operation(device_id, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      self.model_of(device_id))
        # TX/RX frames are now logged centrally by _send / _recv_logged under
        # DEBUG (ROBSTRIDE_DEBUG=1), so no bespoke poll logging is needed here.
        self._send(frame)
        return self._read_status(device_id)
