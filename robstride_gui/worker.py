
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

import math
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

# Run modes that hold a position setpoint. Enabling one must first seed the
# setpoint to the shaft's current position, or the motor snaps to a stale
# target (see ``ControlWorker._seed_hold_position``).
_POSITION_MODES = (RunMode.POSITION_PP, RunMode.POSITION_CSP)

# How far live feedback may exceed the calibrated range before the safety cutout
# disables the motor. A few degrees of slack absorbs position-hold jitter and
# brief overshoot without nuisance trips, while still stopping a runaway before
# it drives an attached part through a hard stop.
RANGE_TRIP_MARGIN_RAD = math.radians(5.0)


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
    torque_ff: Optional[float] = None


@dataclass
class SetSweep(Command):
    """Start/stop a continuous position sweep between two angles.

    While enabled the worker overrides the motor's position setpoint every
    control cycle with a smooth sine that oscillates between ``from_pos`` and
    ``to_pos`` once per ``period`` seconds - a scripted repeating motion for
    soak/simulation. Only takes effect in a position run-mode; angles are in
    radians (user frame), the same units as a normal position setpoint.
    """

    device_id: int
    enabled: bool
    from_pos: float = 0.0
    to_pos: float = 0.0
    period: float = 2.0


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


@dataclass
class SetRangeLimits(Command):
    """Set (or clear) a motor's calibrated travel range in the user frame.

    ``pos_min``/``pos_max`` are radians; ``None`` clears that bound. The worker
    clamps every position-mode setpoint into this range before it reaches the
    bus, so the motor cannot be driven past the calibrated ends.
    """

    device_id: int
    pos_min: Optional[float] = None
    pos_max: Optional[float] = None


@dataclass
class StartRangeCalibration(Command):
    """Enter range-calibration for a motor (LeRobot-style).

    While active the worker records the min/max of the live position. With
    ``make_limp`` the motor is switched to MIT with zero gains/torque so it can
    be moved by hand; the operator may equally jog it - either way the extremes
    reached are captured. The prior run-mode/gains are restored on stop.
    """

    device_id: int
    make_limp: bool = True


@dataclass
class StopRangeCalibration(Command):
    """Finish range-calibration: commit the captured min/max as the range and
    restore the pre-calibration run-mode and gains."""

    device_id: int


# --- board power telemetry ------------------------------------------------------

#: Read VBUS/Iq once every Nth control loop. The control loop runs at 100 Hz; a
#: divisor of 20 polls the board power registers at ~5 Hz, which is plenty for a
#: voltage/current readout while keeping the extra READ_PARAMETER round-trips off
#: the hot path so they never delay a setpoint write.
POWER_READ_DIVISOR: int = 20


# --- communication watchdog ------------------------------------------------------

#: Consecutive transport-level failures in the control loop before the worker
#: declares the link dead and disconnects. At the 100 Hz loop rate this is
#: ~0.1 s of solid failure - long enough to ride out a one-off USB glitch (the
#: transport already retries transient CH340 hiccups internally), short enough
#: that a yanked adapter does not leave enabled motors unsupervised for long.
COMM_FAILURE_LIMIT: int = 10

#: Motor-side watchdog, written to the ``canTimeout`` register (0x7028) on
#: every Enable. A host-side watchdog cannot help when the *host* dies (process
#: crash, cable pull): in velocity/current mode the motor would keep executing
#: its last setpoint forever. With canTimeout armed the motor stops itself when
#: no frame arrives within the window. The worker refreshes setpoints every
#: 10 ms, so 1000 ms has a wide safety margin against false trips. Set the
#: worker's ``motor_can_timeout_ms`` to 0 to skip the write.
#:
#: NOTE: the millisecond unit is assumed from the register spec but has not
#: been verified on this hardware. On first use, enable a motor, stop sending
#: (unplug the adapter) and confirm it actually stops after ~1 s: much sooner
#: or never means the firmware interprets the value differently - adjust here.
MOTOR_CAN_TIMEOUT_MS: int = 1000


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
    torque_ff: float = 0.0   # MIT feed-forward torque (Nm); assist/backdrive comp
    enabled: bool = False
    # Continuous position sweep (soak/simulation): when ``sweep_enabled`` the
    # loop drives position along a sine between ``sweep_from`` and ``sweep_to``
    # once per ``sweep_period`` seconds, timed from ``sweep_t0`` (monotonic).
    sweep_enabled: bool = False
    sweep_from: float = 0.0
    sweep_to: float = 0.0
    sweep_period: float = 2.0
    sweep_t0: float = 0.0


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
    rangeLimitsChanged = Signal(int, object, object)  # device_id, pos_min, pos_max (rad|None)
    motorIdChanged = Signal(int, int)              # old_id, new_id
    zeroStateUpdated = Signal(int, object)         # device_id, ZeroStateInfo
    sweepStopped = Signal(int)                     # device_id: sweep auto-stopped

    def __init__(self, rate_hz: float = 100.0):
        super().__init__()
        self._queue: "queue.Queue[Command]" = queue.Queue()
        self._running = True
        self._bus: Optional[RobstrideBus] = None
        self._targets: dict[int, MotorTarget] = {}
        self._calib: dict[int, Calibration] = {}
        # Per-motor calibrated travel range in the user frame: (min, max) rad;
        # either end may be None (uncalibrated -> no clamp on that side).
        self._range: dict[int, tuple[Optional[float], Optional[float]]] = {}
        # Live range-calibration capture, keyed by device_id. Each entry holds
        # the running (min, max) of observed position plus the run-mode/gains to
        # restore when calibration stops.
        self._range_cal: dict[int, dict] = {}
        self._last_raw_pos: dict[int, float] = {}
        self._safety = SafetyState(SafetyLimits.for_model(proto.DEFAULT_MODEL))
        self._rate_hz = rate_hz
        self._loop_count = 0
        self._comm_failures = 0
        self.motor_can_timeout_ms = MOTOR_CAN_TIMEOUT_MS

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
            self._set_zero(cmd.device_id)
        elif isinstance(cmd, SetMode):
            self._set_mode(cmd.device_id, cmd.mode)
        elif isinstance(cmd, SetTarget):
            self._set_target(cmd)
        elif isinstance(cmd, SetSweep):
            self._set_sweep(cmd)
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
        elif isinstance(cmd, SetRangeLimits):
            self._set_range_limits(cmd.device_id, cmd.pos_min, cmd.pos_max)
        elif isinstance(cmd, StartRangeCalibration):
            self._start_range_calibration(cmd.device_id, cmd.make_limp)
        elif isinstance(cmd, StopRangeCalibration):
            self._stop_range_calibration(cmd.device_id)

    def _connect(self, cmd: Connect) -> None:
        self._comm_failures = 0
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
            self._comm_failures = 0
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
        # Hard safety latch: never energise a motor while E-STOP is engaged. The
        # control loop already skips commanding an estopped motor, but enabling
        # still applies holding torque - the operator expects a dead motor.
        if self._safety.estop:
            self.error.emit(
                f"M{device_id}: E-STOP engaged - clear it before enabling")
            self.motorEnabledChanged.emit(device_id, False)
            return
        # Assert the run-mode before enabling: the motor keeps its previous mode
        # across power cycles, so without this it may sit in MIT mode and ignore
        # every position/velocity/current setpoint we send.
        self._bus.set_run_mode(device_id, target.mode)
        # Safe-enable: seed the setpoint to the shaft's *current* position so the
        # motor comes up HOLDING where it is. Without this it snaps to the last
        # (default 0.0) position target the instant it is enabled, which can slam
        # attached hardware into its surroundings. Only position/MIT modes hold a
        # position; velocity/current default to 0 (no motion) and need no seed.
        if target.mode in _POSITION_MODES or target.mode == RunMode.MIT:
            self._seed_hold_position(device_id, target)
        self._configure_motor_watchdog(device_id)
        self._bus.enable(device_id)
        target.enabled = True
        # Re-assert the hold setpoint AFTER enabling. The pre-enable seed above is
        # a belt-and-suspenders: RobStride ignores a loc_ref (POSITION_TARGET)
        # write while the motor is disabled, so on its own it keeps the internal
        # target from the *previous* enable and profiles straight back to that old
        # angle the instant it is energised (the "jumps to the last position, not
        # the new zero" bug - most visible after a Set Zero at a hand-moved spot).
        # Writing loc_ref once more here, now that the motor accepts it, pins the
        # hold to the shaft's current spot and closes the window before the first
        # control-loop tick.
        if target.mode in _POSITION_MODES:
            raw = self._last_raw_pos.get(device_id)
            if raw is not None:
                self._bus.set_position(device_id, raw, self._safety.limits.velocity_max)
        self.motorEnabledChanged.emit(device_id, True)
        self.log.emit(f"M{device_id}: enabled (mode {RunMode.NAMES.get(target.mode, target.mode)})")

    def _seed_hold_position(self, device_id: int, target: MotorTarget) -> None:
        """Make the shaft's current position the hold setpoint before enabling.

        The reading comes from ``poll_status`` so it is in the SAME frame as the
        live feedback and the motor's mechanical zero. A plain ``mechPos`` param
        read can lag a just-applied Set Zero: it would seed the setpoint in the
        old frame and swing the shaft back to its pre-zero position on enable.
        The motor is always disabled at this point, so the zero-gain status frame
        commands no motion. For profiled/CSP modes we also pre-load ``loc_ref``
        so the first hold is the current spot. A failed read is surfaced loudly:
        enabling blind risks the jump this guards against.
        """
        status = self._bus.poll_status(device_id)
        if status is None:
            self.error.emit(
                f"M{device_id}: could not read position before enable - the "
                "motor may jump to its last target. Check the connection.")
            return
        raw = float(status.position)
        c = self._calib.get(device_id) or Calibration()
        self._last_raw_pos[device_id] = raw
        target.position = c.pos_from_raw(raw)
        if target.mode in _POSITION_MODES:
            self._bus.set_position(device_id, raw, self._safety.limits.velocity_max)

    def _disable(self, device_id: int) -> None:
        if not self._bus:
            return
        self._bus.disable(device_id)
        target = self._targets.get(device_id)
        if target is not None:
            target.enabled = False
            # Stop any sweep so the next enable holds position instead of
            # resuming the trajectory - that would defeat safe-enable.
            self._stop_sweep(device_id, target)
        self.motorEnabledChanged.emit(device_id, False)
        self.log.emit(f"M{device_id}: disabled")

    def _set_zero(self, device_id: int) -> None:
        """Rewrite the motor's mechanical zero at the current spot and persist it.

        SET_ZERO_POSITION is meant for a *stopped* motor: issuing it while a
        position loop is actively holding shifts the encoder frame out from under
        the setpoint, so the shaft twitches, and the ~0.25 s save (see
        ``RobstrideBus.set_zero``) freezes the readout mid-move - the operator
        sees the feedback wander instead of snapping to 0. So if the motor is
        enabled we disable it around the zero, then re-seed the hold at the new
        zero and re-enable: the shaft stays put and the readout lands cleanly on
        0. If it was already disabled this is a plain zero with no motion.
        """
        if self._bus is None:
            return
        target = self._targets.get(device_id)
        was_enabled = target is not None and target.enabled
        # Stop the active position loop before redefining its reference frame.
        if was_enabled:
            self._disable(device_id)
        self._bus.set_zero(device_id)
        self.log.emit(f"M{device_id}: zero set and saved to flash")
        # A hardware zero remakes the motor's mechanical frame at the current
        # spot. Any software offset would now double-count, so the readout would
        # show -offset instead of 0. Clear it (keep the direction/invert) so the
        # two frames agree; the change is persisted via calibrationChanged.
        calib = self._calib.setdefault(device_id, Calibration())
        calib.offset = 0.0
        self.calibrationChanged.emit(device_id, calib.direction, 0.0)
        # The new zero is the current spot, so any stored hold setpoint (e.g. one
        # frozen mid-swing on disable) is now stale and would swing the shaft back
        # there on the next enable. Reset it to the new zero.
        if target is not None:
            target.position = 0.0
        # Refresh the readout once so the new zero shows immediately. The motor is
        # disabled at this point (either it already was, or we just disabled it),
        # so poll_status is safe - it will not inject a brake frame into a live
        # control loop.
        status = self._bus.poll_status(device_id)
        if status is not None:
            self.statusUpdated.emit(
                device_id, self._decalibrate(device_id, status, calib))
        # Read the motor's own zero back so the UI confirms it stuck.
        self._read_zero_state(device_id)
        # Bring the motor back up holding the new zero if it was running before;
        # _enable re-seeds the hold from the (now ~0) live position, so the shaft
        # stays put rather than jumping.
        if was_enabled:
            self._enable(device_id)

    def _stop_sweep(self, device_id: int, target: MotorTarget) -> None:
        """Clear a running sweep, freezing its setpoint where it left off. Emits
        ``sweepStopped`` so the UI's sweep button clears too. No-op if idle."""
        if not target.sweep_enabled:
            return
        target.position = self._sweep_position(target)
        target.sweep_enabled = False
        self.sweepStopped.emit(device_id)

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
            # Safe-enable on the re-enable too: switching an enabled motor into a
            # position mode would otherwise snap it to the stale setpoint. Seed
            # the current position first, exactly as _enable does.
            if mode in _POSITION_MODES or mode == RunMode.MIT:
                self._seed_hold_position(device_id, target)
            # Re-arm the motor-side watchdog too: this disable/enable pulse is
            # a full enable, and firmware may not keep canTimeout across it.
            self._configure_motor_watchdog(device_id)
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
        if cmd.torque_ff is not None:
            t.torque_ff = cmd.torque_ff

    def _set_sweep(self, cmd: SetSweep) -> None:
        t = self._targets.setdefault(cmd.device_id, MotorTarget())
        t.sweep_from = cmd.from_pos
        t.sweep_to = cmd.to_pos
        # Guard the divisor: a zero/negative period would divide by zero in the
        # sine phase. 50 ms is already far faster than any real motor can track.
        t.sweep_period = max(float(cmd.period), 0.05)
        t.sweep_enabled = cmd.enabled
        if cmd.enabled:
            t.sweep_t0 = time.monotonic()
            self.log.emit(
                f"M{cmd.device_id}: sweep {math.degrees(cmd.from_pos):+.1f} -> "
                f"{math.degrees(cmd.to_pos):+.1f} deg, period {t.sweep_period:.2f}s")
        else:
            # Freeze the setpoint where the sweep left off so the motor holds
            # instead of jumping back to a stale manual position.
            t.position = self._sweep_position(t)
            self.log.emit(f"M{cmd.device_id}: sweep stopped")

    @staticmethod
    def _sweep_position(t: MotorTarget) -> float:
        """Sine setpoint for the current time: ``from`` at t0, ``to`` at the
        half-period, back to ``from`` at the full period. Continuous in velocity
        (no snap at the endpoints), which is what makes it smooth."""
        mid = (t.sweep_from + t.sweep_to) / 2.0
        amp = (t.sweep_to - t.sweep_from) / 2.0
        phase = (time.monotonic() - t.sweep_t0) / t.sweep_period
        return mid - amp * math.cos(2.0 * math.pi * phase)

    def _estop(self, engage: bool) -> None:
        if engage:
            self._safety.engage_estop()
            self.log.emit("E-STOP engaged")
            if self._bus:
                for device_id, target in self._targets.items():
                    self._stop_sweep(device_id, target)
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
        for table in (self._targets, self._calib, self._range,
                      self._range_cal, self._last_raw_pos):
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

    # -- range calibration -------------------------------------------------------

    def _set_range_limits(self, device_id: int, pos_min: Optional[float],
                          pos_max: Optional[float]) -> None:
        """Store a motor's travel range, normalising so min <= max.

        An inverted-direction motor can report the two ends in either order; sort
        them so the clamp always has a well-formed [lo, hi]. Either end may stay
        ``None`` to leave that side unbounded.
        """
        lo = None if pos_min is None else float(pos_min)
        hi = None if pos_max is None else float(pos_max)
        if lo is not None and hi is not None and lo > hi:
            lo, hi = hi, lo
        self._range[device_id] = (lo, hi)
        self.rangeLimitsChanged.emit(device_id, lo, hi)
        span = ("unbounded" if lo is None and hi is None
                else f"[{'-inf' if lo is None else f'{lo:.3f}'}, "
                     f"{'+inf' if hi is None else f'{hi:.3f}'}] rad")
        self.log.emit(f"M{device_id}: range limits {span}")

    def _start_range_calibration(self, device_id: int, make_limp: bool) -> None:
        t = self._targets.setdefault(device_id, MotorTarget())
        state = {"min": None, "max": None, "prev_mode": t.mode,
                 "prev_kp": t.kp, "prev_kd": t.kd, "prev_tq": t.torque_ff,
                 "limp": make_limp}
        # Seed from the current position if we already have feedback, so a motor
        # sitting still still yields a (degenerate) range rather than nothing.
        c = self._calib.get(device_id) or Calibration()
        raw = self._last_raw_pos.get(device_id)
        if raw is not None:
            user = c.pos_from_raw(raw)
            state["min"] = state["max"] = user
        self._range_cal[device_id] = state
        if make_limp:
            # MIT with zero gains/torque = no holding effort, so the shaft can be
            # moved by hand. Goes through _set_mode so an enabled motor is cleanly
            # re-armed in the new mode.
            self._set_mode(device_id, RunMode.MIT)
            t.kp = 0.0
            t.kd = 0.0
            t.torque_ff = 0.0
        self.log.emit(f"M{device_id}: range calibration started - move the motor "
                      "through its travel (by hand or jog), then Stop")

    def _stop_range_calibration(self, device_id: int) -> None:
        state = self._range_cal.pop(device_id, None)
        if state is None:
            return
        t = self._targets.get(device_id)
        if t is not None and state.get("limp"):
            # Restore the gains we zeroed, then the run-mode we came from.
            t.kp = state["prev_kp"]
            t.kd = state["prev_kd"]
            t.torque_ff = state["prev_tq"]
            self._set_mode(device_id, state["prev_mode"])
        lo, hi = state["min"], state["max"]
        if lo is None or hi is None:
            self.log.emit(f"M{device_id}: range calibration stopped - no motion "
                          "captured (enable the motor and move it), range unchanged")
            return
        self._set_range_limits(device_id, lo, hi)

    def _note_range_sample(self, device_id: int, user_pos: float) -> None:
        """Fold one live position sample into an active calibration's min/max."""
        state = self._range_cal.get(device_id)
        if state is None:
            return
        if state["min"] is None or user_pos < state["min"]:
            state["min"] = user_pos
        if state["max"] is None or user_pos > state["max"]:
            state["max"] = user_pos

    def _clamp_to_range(self, device_id: int, value: float) -> float:
        lo, hi = self._range.get(device_id, (None, None))
        if lo is not None and value < lo:
            return lo
        if hi is not None and value > hi:
            return hi
        return value

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

    def _configure_motor_watchdog(self, device_id: int) -> None:
        """Arm the motor's own CAN watchdog before enabling it.

        Writes ``motor_can_timeout_ms`` to the canTimeout register (0x7028) so
        the motor stops itself if the host goes silent - the failure a
        host-side watchdog cannot cover. Best-effort: a motor that does not
        ack the write still enables, but the gap is logged so the operator
        knows the safety net is missing.
        """
        timeout_ms = int(self.motor_can_timeout_ms)
        if timeout_ms <= 0:
            return
        ack = self._bus.write_param(device_id, ParameterType.CAN_TIMEOUT, timeout_ms)
        if ack is None:
            # error (not just log): an unarmed motor-side watchdog is the most
            # safety-relevant gap here - the motor will NOT stop on its own if
            # the link to the host drops. The operator must see this.
            self.error.emit(
                f"M{device_id}: canTimeout write not acked - the motor will "
                "NOT stop on its own if the link to the host drops")

    # -- host-side communication watchdog -----------------------------------------

    def _note_comm_failure(self, message: str) -> None:
        """Count a transport-level failure; disconnect once the limit is hit.

        Only the first failure of a burst is surfaced (the UI would otherwise
        get one error per motor per 100 Hz cycle). Reaching
        :data:`COMM_FAILURE_LIMIT` consecutive failures means the link is dead,
        not glitching: tear down instead of spinning on it while enabled motors
        run unsupervised - their own canTimeout watchdog stops them.
        """
        self._comm_failures += 1
        if self._comm_failures == 1:
            self.error.emit(message)
        elif self._comm_failures >= COMM_FAILURE_LIMIT:
            self.error.emit(
                f"Connection lost: {self._comm_failures} consecutive bus "
                f"failures (last: {message}). Disconnecting - enabled motors "
                "stop via their canTimeout watchdog.")
            self._teardown()

    # -- per-loop servicing ------------------------------------------------------

    def _service_motors(self) -> None:
        read_power = self._loop_count % POWER_READ_DIVISOR == 0
        # The failure counter may only reset after a *fully clean* pass. If it
        # were reset on any motor's success, one healthy motor would clear the
        # count that a dead one keeps accruing in the same cycle, and the
        # watchdog could never trip (nor its error dedup engage).
        failures_before = self._comm_failures
        for device_id, target in list(self._targets.items()):
            if self._bus is None:
                return  # the watchdog tore the connection down mid-iteration
            if not target.enabled or self._safety.estop:
                continue
            try:
                status = self._command_motor(device_id, target)
            except TransportError as e:
                self._note_comm_failure(str(e))
                continue
            if status is not None:
                self.statusUpdated.emit(device_id, status)
                self._enforce_range_cutout(device_id, target, status.position)
            if read_power:
                self._read_power(device_id)
        if self._bus is not None and self._comm_failures == failures_before:
            self._comm_failures = 0  # clean pass: the link is healthy again

    def _read_power(self, device_id: int) -> None:
        """Read the board's bus voltage and current and emit derived power.

        These are plain READ_PARAMETER round-trips (never a motion frame), so
        they are safe to interleave with the control loop without disturbing the
        motor. Done at ~5 Hz via :data:`POWER_READ_DIVISOR` to keep them off the
        hot path. A non-responding read leaves the value out and emits nothing.
        """
        if self._bus is None:
            return
        try:
            vbus = self._bus.read_param(device_id, ParameterType.VBUS)
            iq = self._bus.read_param(device_id, ParameterType.IQ_FILTERED)
        except TransportError as e:
            self._note_comm_failure(str(e))
            return
        if vbus is None or iq is None:
            return
        vbus = float(vbus)
        iq = float(iq)
        self.powerUpdated.emit(device_id, PowerInfo(device_id, vbus, iq, vbus * iq))

    def _enforce_range_cutout(self, device_id: int, target: MotorTarget,
                              user_pos: float) -> None:
        """Disable the motor if live feedback leaves the calibrated range.

        Position/MIT setpoints are already clamped to the range, but velocity and
        current modes have NO position clamp - this feedback-driven cutout is the
        only thing that stops the shaft driving through an end-stop in those
        modes. Skipped while range-calibrating, since that deliberately moves the
        shaft past the old bounds to redefine them.
        """
        if device_id in self._range_cal:
            return
        lo, hi = self._range.get(device_id, (None, None))
        if lo is None and hi is None:
            return
        m = RANGE_TRIP_MARGIN_RAD
        if (lo is not None and user_pos < lo - m) or \
           (hi is not None and user_pos > hi + m):
            try:
                self._bus.disable(device_id)
            except Exception:
                pass
            target.enabled = False
            self._stop_sweep(device_id, target)
            self.motorEnabledChanged.emit(device_id, False)
            self.error.emit(
                f"M{device_id}: position {math.degrees(user_pos):+.1f} deg left "
                "the calibrated range - motor disabled for safety")

    def _command_motor(self, device_id: int, t: MotorTarget) -> Optional[MotorStatus]:
        """Clamp the target in the *user* frame, convert to the *raw* motor frame
        via the motor's calibration, send it, and de-calibrate the reply back to
        the user frame for display."""
        s = self._safety
        c = self._calib.get(device_id) or Calibration()
        # A running sweep drives the position setpoint itself, but only in a
        # position mode - in velocity/current/MIT the position field is unused,
        # so leave it alone. Writing t.position keeps the readout/graph honest.
        if t.sweep_enabled and t.mode in (RunMode.POSITION_PP, RunMode.POSITION_CSP):
            t.position = self._sweep_position(t)
        if t.mode == RunMode.MIT:
            raw = self._bus.operation(
                device_id,
                c.pos_to_raw(self._clamp_to_range(
                    device_id, s.clamp_position(t.position))),
                c.signed_to_raw(s.clamp_velocity(t.velocity)),
                s.clamp_kp(t.kp),
                s.clamp_kd(t.kd),
                s.clamp_torque(t.torque_ff),
            )
            return self._decalibrate(device_id, raw, c)
        if t.mode in (RunMode.POSITION_PP, RunMode.POSITION_CSP):
            status = self._bus.set_position(
                device_id, c.pos_to_raw(self._clamp_to_range(
                    device_id, s.clamp_position(t.position))),
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
        user_pos = c.pos_from_raw(status.position)
        # While range-calibrating, every live sample widens the captured min/max.
        self._note_range_sample(device_id, user_pos)
        return replace(
            status,
            position=user_pos,
            velocity=c.signed_from_raw(status.velocity),
            torque=c.signed_from_raw(status.torque),
        )
