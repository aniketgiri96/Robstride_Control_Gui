
"""Background control worker.

The GUI thread must never block on serial / CAN IO, so all bus interaction
happens here on a dedicated :class:`QThread`. The UI talks to the worker by
pushing :class:`Command` objects onto a thread-safe queue; the worker emits Qt
signals back with connection state, per-motor status, and log/error messages.

Per-motor *targets* (mode + setpoints + gains + enabled flag) live in a small
table. Each loop iteration the worker drains pending commands, then for every
enabled motor issues the appropriate frame for its current run-mode and emits
the decoded feedback.
"""

from __future__ import annotations

import queue
import time
from dataclasses import dataclass, replace
from typing import Optional

from PySide6.QtCore import QObject, Signal

from . import protocol as proto
from .bus import BusConfig, Motor, RobstrideBus
from .protocol import MotorStatus, ParameterType, RunMode
from .safety import Calibration, SafetyLimits, SafetyState
from .transport import Transport, TransportError


# --- command objects pushed from the UI -----------------------------------------


@dataclass
class Command:
    """Base class for UI -> worker requests."""


@dataclass
class Connect(Command):
    transport: Transport
    motors: list[Motor]


@dataclass
class AddMotor(Command):
    """Register a motor added/discovered after Connect so it is polled and driveable.

    Connect seeds the bus from the tabs open at connect time; a motor added later
    (e.g. one surfaced by Detect) needs the same registration or it has a tab but
    never gets serviced. Idempotent: re-registering a known id is harmless.
    """

    device_id: int
    model: str = proto.DEFAULT_MODEL


@dataclass
class Disconnect(Command):
    pass


@dataclass
class Scan(Command):
    start: int = 1
    end: int = 16


@dataclass
class Inventory(Command):
    """List responding CAN ids and each motor's unique MCU id, for the device panel."""

    start: int = 1
    end: int = 16


@dataclass
class Enable(Command):
    device_id: int


@dataclass
class Disable(Command):
    device_id: int


@dataclass
class SetZero(Command):
    device_id: int


@dataclass
class SetMode(Command):
    device_id: int
    mode: int


@dataclass
class SetTarget(Command):
    """Update one or more setpoints for a motor (None = leave unchanged)."""

    device_id: int
    position: Optional[float] = None
    velocity: Optional[float] = None
    current: Optional[float] = None
    kp: Optional[float] = None
    kd: Optional[float] = None


@dataclass
class EStop(Command):
    engage: bool


@dataclass
class SetLimits(Command):
    limits: SafetyLimits


@dataclass
class SetCalibration(Command):
    device_id: int
    direction: int = 1     # +1 normal, -1 inverted
    offset: float = 0.0    # rad, raw frame


@dataclass
class CaptureZero(Command):
    """Set this motor's zero-offset to its current measured position."""

    device_id: int


@dataclass
class SetMotorId(Command):
    """Reassign a motor's CAN id (only one motor should be on the bus)."""

    current_id: int
    new_id: int


@dataclass
class ReadZeroState(Command):
    """Read the motor's own persisted zero markers, to show if it remembers zero."""

    device_id: int


# --- board power telemetry ------------------------------------------------------

#: Read VBUS/Iq once every Nth control loop. The control loop runs at 100 Hz; a
#: divisor of 20 polls the board power registers at ~5 Hz, which is plenty for a
#: voltage/current readout while keeping the extra READ_PARAMETER round-trips off
#: the hot path so they never delay a setpoint write.
POWER_READ_DIVISOR: int = 20


@dataclass(frozen=True)
class PowerInfo:
    """Electrical telemetry read from the motor control board."""

    device_id: int
    vbus: float       # bus voltage, V
    iq: float         # filtered q-axis current, A
    power: float      # estimated input power VBUS*Iq, W


@dataclass(frozen=True)
class ZeroStateInfo:
    """The motor's own persisted zero markers, read back from its registers.

    ``mech_offset`` is the stored mechanical zero (``mechOffset``, 0x2005) the
    motor keeps in flash; ``zero_sta`` is its zero-state flag (0x7029). Together
    they show whether the motor remembers its absolute zero. Either field is
    ``None`` if that register did not answer.
    """

    device_id: int
    zero_sta: Optional[int]
    mech_offset: Optional[float]


# --- per-motor live target ------------------------------------------------------


@dataclass
class MotorTarget:
    mode: int = RunMode.POSITION_PP
    position: float = 0.0
    velocity: float = 0.0
    current: float = 0.0
    kp: float = 28.0
    kd: float = 6.0
    enabled: bool = False


class ControlWorker(QObject):
    """Runs the control loop; lives in its own QThread."""

    statusUpdated = Signal(int, object)   # device_id, MotorStatus
    powerUpdated = Signal(int, object)    # device_id, PowerInfo
    connectionChanged = Signal(bool)
    scanFinished = Signal(list)           # list[int]
    busCollision = Signal(list)           # list[int]: ids answered by >1 motor
    inventoryReady = Signal(list)         # list[dict]: {"can_id", "uids"} per id
    log = Signal(str)
    error = Signal(str)
    motorEnabledChanged = Signal(int, bool)
    calibrationChanged = Signal(int, int, float)   # device_id, direction, offset
    motorIdChanged = Signal(int, int)              # old_id, new_id
    zeroStateUpdated = Signal(int, object)         # device_id, ZeroStateInfo

    def __init__(self, rate_hz: float = 100.0):
        super().__init__()
        self._queue: "queue.Queue[Command]" = queue.Queue()
        self._running = True
        self._bus: Optional[RobstrideBus] = None
        self._targets: dict[int, MotorTarget] = {}
        self._calib: dict[int, Calibration] = {}
        self._last_raw_pos: dict[int, float] = {}
        self._safety = SafetyState(SafetyLimits.for_model(proto.DEFAULT_MODEL))
        self._rate_hz = rate_hz
        self._loop_count = 0

    # -- public, thread-safe API (called from the GUI thread) --------------------

    def post(self, command: Command) -> None:
        self._queue.put(command)

    def stop(self) -> None:
        self._running = False
        self._queue.put(Disconnect())

    # -- main loop ---------------------------------------------------------------

    def run(self) -> None:
        period = 1.0 / self._rate_hz
        while self._running:
            t0 = time.monotonic()
            self._loop_count += 1
            self._drain_commands()
            if self._bus is not None and self._bus.is_open:
                self._service_motors()
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)
        self._teardown()

    # -- command handling --------------------------------------------------------

    def _drain_commands(self) -> None:
        while True:
            try:
                cmd = self._queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._apply(cmd)
            except TransportError as e:
                self.error.emit(str(e))
            except Exception as e:  # never let one bad command kill the loop
                self.error.emit(f"{type(e).__name__}: {e}")

    def _apply(self, cmd: Command) -> None:
        if isinstance(cmd, Connect):
            self._connect(cmd)
        elif isinstance(cmd, AddMotor):
            self._add_motor(cmd.device_id, cmd.model)
        elif isinstance(cmd, Disconnect):
            self._teardown()
        elif isinstance(cmd, Scan):
            self._scan(cmd)
        elif isinstance(cmd, Inventory):
            self._inventory(cmd)
        elif isinstance(cmd, Enable):
            self._enable(cmd.device_id)
        elif isinstance(cmd, Disable):
            self._disable(cmd.device_id)
        elif isinstance(cmd, SetZero):
            if self._bus:
                self._bus.set_zero(cmd.device_id)
                self.log.emit(f"M{cmd.device_id}: zero set and saved to flash")
                # Read the motor's own zero back so the UI confirms it stuck.
                self._read_zero_state(cmd.device_id)
        elif isinstance(cmd, SetMode):
            self._set_mode(cmd.device_id, cmd.mode)
        elif isinstance(cmd, SetTarget):
            self._set_target(cmd)
        elif isinstance(cmd, EStop):
            self._estop(cmd.engage)
        elif isinstance(cmd, SetLimits):
            self._safety.limits = cmd.limits
        elif isinstance(cmd, SetCalibration):
            self._calib[cmd.device_id] = Calibration(int(cmd.direction), float(cmd.offset))
            self.log.emit(f"M{cmd.device_id}: direction={'inverted' if cmd.direction < 0 else 'normal'}, "
                          f"offset={cmd.offset:.3f} rad")
        elif isinstance(cmd, CaptureZero):
            self._capture_zero(cmd.device_id)
        elif isinstance(cmd, SetMotorId):
            self._set_motor_id(cmd.current_id, cmd.new_id)
        elif isinstance(cmd, ReadZeroState):
            self._read_zero_state(cmd.device_id)

    def _connect(self, cmd: Connect) -> None:
        self._bus = RobstrideBus(cmd.transport, BusConfig())
        for m in cmd.motors:
            self._bus.add_motor(m)
            self._targets.setdefault(m.device_id, MotorTarget())
        self._bus.open()
        self.connectionChanged.emit(True)
        self.log.emit(f"Connected via {cmd.transport.name}")

    def _add_motor(self, device_id: int, model: str) -> None:
        if self._bus is None:
            return
        self._bus.add_motor(Motor(device_id=device_id, model=model))
        self._targets.setdefault(device_id, MotorTarget())

    def _teardown(self) -> None:
        if self._bus is not None:
            for device_id, target in self._targets.items():
                if target.enabled:
                    try:
                        self._bus.disable(device_id)
                    except Exception:
                        pass
                    target.enabled = False
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None
            self.connectionChanged.emit(False)
            self.log.emit("Disconnected")

    def _scan(self, cmd: Scan) -> None:
        if not self._bus:
            return
        found = self._bus.scan(cmd.start, cmd.end)
        self.scanFinished.emit(found)
        self.log.emit(f"Scan found motors: {found or 'none'}")
        collisions = self._bus.find_collisions(found)
        if collisions:
            self.busCollision.emit(collisions)
            ids = ", ".join(str(d) for d in collisions)
            self.log.emit(f"WARNING: CAN id collision on {ids} - "
                          "multiple motors share an id and move together")

    def _inventory(self, cmd: Inventory) -> None:
        if not self._bus:
            return
        items = self._bus.inventory(cmd.start, cmd.end)
        payload = [{"can_id": did, "uids": [u.hex() for u in uids]}
                   for did, uids in items]
        self.inventoryReady.emit(payload)
        self.log.emit(f"Device inventory: {len(payload)} id(s) responding")

    def _enable(self, device_id: int) -> None:
        if not self._bus:
            return
        target = self._targets.setdefault(device_id, MotorTarget())
        # Assert the run-mode before enabling: the motor keeps its previous mode
        # across power cycles, so without this it may sit in MIT mode and ignore
        # every position/velocity/current setpoint we send.
        self._bus.set_run_mode(device_id, target.mode)
        self._bus.enable(device_id)
        target.enabled = True
        self.motorEnabledChanged.emit(device_id, True)
        self.log.emit(f"M{device_id}: enabled (mode {RunMode.NAMES.get(target.mode, target.mode)})")

    def _disable(self, device_id: int) -> None:
        if not self._bus:
            return
        self._bus.disable(device_id)
        if device_id in self._targets:
            self._targets[device_id].enabled = False
        self.motorEnabledChanged.emit(device_id, False)
        self.log.emit(f"M{device_id}: disabled")

    def _set_mode(self, device_id: int, mode: int) -> None:
        if not self._bus:
            return
        target = self._targets.setdefault(device_id, MotorTarget())
        # A run-mode switch is only reliably accepted while the motor is
        # disabled; bracket the change so an enabled motor ends up enabled again
        # in the new mode (this is what makes the jog buttons work).
        was_enabled = target.enabled
        if was_enabled:
            self._bus.disable(device_id)
        self._bus.set_run_mode(device_id, mode)
        target.mode = mode
        if was_enabled:
            self._bus.enable(device_id)
        self.log.emit(f"M{device_id}: mode -> {RunMode.NAMES.get(mode, mode)}")

    def _set_target(self, cmd: SetTarget) -> None:
        t = self._targets.setdefault(cmd.device_id, MotorTarget())
        if cmd.position is not None:
            t.position = cmd.position
        if cmd.velocity is not None:
            t.velocity = cmd.velocity
        if cmd.current is not None:
            t.current = cmd.current
        if cmd.kp is not None:
            t.kp = cmd.kp
        if cmd.kd is not None:
            t.kd = cmd.kd

    def _estop(self, engage: bool) -> None:
        if engage:
            self._safety.engage_estop()
            self.log.emit("E-STOP engaged")
            if self._bus:
                for device_id, target in self._targets.items():
                    if target.enabled:
                        try:
                            self._bus.disable(device_id)
                        except Exception:
                            pass
                        target.enabled = False
                        self.motorEnabledChanged.emit(device_id, False)
        else:
            self._safety.clear_estop()
            self.log.emit("E-STOP cleared")

    def _set_motor_id(self, current_id: int, new_id: int) -> None:
        if not self._bus:
            self.error.emit("Connect before changing a motor id")
            return
        if new_id == current_id:
            return
        live_id = self._bus.set_motor_id(current_id, new_id)
        if live_id is None:
            self.error.emit(f"Motor {current_id} did not respond after the id change. "
                            "Power-cycle the motor and press Detect.")
            return
        if live_id == current_id:
            # The motor still answers at its old id: the new id is not active
            # yet. Leave the panel where it is so live feedback keeps flowing.
            self.log.emit(f"Motor still at ID {current_id}: new id not active yet. "
                          "Power-cycle the motor, then Detect.")
            return
        # Follow the motor to the id it actually responds at now.
        for table in (self._targets, self._calib, self._last_raw_pos):
            if current_id in table:
                table[live_id] = table.pop(current_id)
        self.motorIdChanged.emit(current_id, live_id)
        target = self._targets.get(live_id)
        if target is not None and target.enabled:
            self.motorEnabledChanged.emit(live_id, True)
        self.log.emit(f"Motor {current_id} -> ID {live_id} (verified and saved).")

    def _capture_zero(self, device_id: int) -> None:
        raw = self._last_raw_pos.get(device_id)
        if raw is None:
            self.log.emit(f"M{device_id}: no feedback yet - enable the motor first")
            return
        calib = self._calib.setdefault(device_id, Calibration())
        calib.offset = raw
        self.calibrationChanged.emit(device_id, calib.direction, calib.offset)
        self.log.emit(f"M{device_id}: zero captured at {raw:.3f} rad")

    def _read_zero_state(self, device_id: int) -> None:
        """Read the motor's persisted zero markers and emit them for display."""
        if not self._bus:
            return
        info = self._bus.read_zero_state(device_id)
        if info is None:
            self.log.emit(f"M{device_id}: zero state unavailable (no reply)")
            return
        self.zeroStateUpdated.emit(
            device_id, ZeroStateInfo(device_id, info["zero_sta"], info["mech_offset"]))
        self.log.emit(f"M{device_id}: motor zero_sta={info['zero_sta']}, "
                      f"mechOffset={info['mech_offset']}")

    # -- per-loop servicing ------------------------------------------------------

    def _service_motors(self) -> None:
        read_power = self._loop_count % POWER_READ_DIVISOR == 0
        for device_id, target in list(self._targets.items()):
            if not target.enabled or self._safety.estop:
                continue
            try:
                status = self._command_motor(device_id, target)
            except TransportError as e:
                self.error.emit(str(e))
                continue
            if status is not None:
                self.statusUpdated.emit(device_id, status)
            if read_power:
                self._read_power(device_id)

    def _read_power(self, device_id: int) -> None:
        """Read the board's bus voltage and current and emit derived power.

        These are plain READ_PARAMETER round-trips (never a motion frame), so
        they are safe to interleave with the control loop without disturbing the
        motor. Done at ~5 Hz via :data:`POWER_READ_DIVISOR` to keep them off the
        hot path. A non-responding read leaves the value out and emits nothing.
        """
        try:
            vbus = self._bus.read_param(device_id, ParameterType.VBUS)
            iq = self._bus.read_param(device_id, ParameterType.IQ_FILTERED)
        except TransportError as e:
            self.error.emit(str(e))
            return
        if vbus is None or iq is None:
            return
        vbus = float(vbus)
        iq = float(iq)
        self.powerUpdated.emit(device_id, PowerInfo(device_id, vbus, iq, vbus * iq))

    def _command_motor(self, device_id: int, t: MotorTarget) -> Optional[MotorStatus]:
        """Clamp the target in the *user* frame, convert to the *raw* motor frame
        via the motor's calibration, send it, and de-calibrate the reply back to
        the user frame for display."""
        s = self._safety
        c = self._calib.get(device_id) or Calibration()
        if t.mode == RunMode.MIT:
            raw = self._bus.operation(
                device_id,
                c.pos_to_raw(s.clamp_position(t.position)),
                c.signed_to_raw(s.clamp_velocity(t.velocity)),
                s.clamp_kp(t.kp),
                s.clamp_kd(t.kd),
                0.0,
            )
            return self._decalibrate(device_id, raw, c)
        if t.mode in (RunMode.POSITION_PP, RunMode.POSITION_CSP):
            status = self._bus.set_position(
                device_id, c.pos_to_raw(s.clamp_position(t.position)),
                s.limits.velocity_max)
        elif t.mode == RunMode.VELOCITY:
            status = self._bus.set_velocity(
                device_id, c.signed_to_raw(s.clamp_velocity(t.velocity)),
                s.limits.current_max)
        elif t.mode == RunMode.CURRENT:
            status = self._bus.set_current(
                device_id, c.signed_to_raw(s.clamp_current(t.current)))
        else:
            status = None
        # Feedback comes from the status frame the setpoint write is acked with.
        # Do NOT call poll_status here: it sends an MIT operation_control frame
        # which, in position/velocity/current mode, commands zero and brakes the
        # motor every control cycle - the motor would twitch and never sustain
        # motion. (This was the root cause of "motors don't rotate".)
        return self._decalibrate(device_id, status, c)

    def _decalibrate(self, device_id: int, status: Optional[MotorStatus],
                     c: Calibration) -> Optional[MotorStatus]:
        """Record the raw position (for capture-zero) and convert feedback to the
        user frame for display."""
        if status is None:
            return None
        self._last_raw_pos[device_id] = status.position
        return replace(
            status,
            position=c.pos_from_raw(status.position),
            velocity=c.signed_from_raw(status.velocity),
            torque=c.signed_from_raw(status.torque),
        )
