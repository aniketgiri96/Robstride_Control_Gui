"""Live MuJoCo <-> motor bridge.

The dashboard already replays a *recorded* joint log offline
(:mod:`robstride_gui.jointlog` -> :mod:`robstride_gui.sequence` ->
:class:`~robstride_gui.ui.sequencer.SequencePlayer`). This module is the *live*
counterpart: it streams MuJoCo's current joint angles onto the motors in real
time and hands the motors' measured state back to MuJoCo, so the physical robot
mirrors the simulated one (or vice-versa).

It plugs into the existing control path at exactly two points and knows nothing
else about the worker internals:

* **out (sim -> motor):** ``worker.post(SetTarget(device_id, position=rad))`` -
  the same command the jog buttons and the sequencer use. The 100 Hz control
  loop clamps it through the safety limits and the motor's calibrated range,
  converts user-frame -> raw, and puts it on the wire. Posting is thread-safe
  (the worker drains a queue), so :meth:`push_targets` may be called from a
  MuJoCo thread, a Qt timer, or a socket receive thread.
* **in (motor -> sim):** the worker's ``statusUpdated(device_id, MotorStatus)``
  signal. :meth:`latest_state` returns the newest measured angle/vel/torque per
  joint for MuJoCo to read.

Two ways to drive it:

* **in-process** - if MuJoCo runs inside the GUI process, hold a
  :class:`MujocoBridge` and call :meth:`push_targets` / :meth:`latest_state`
  directly (e.g. from a ``QTimer``).
* **out-of-process** - if MuJoCo is its own script (the usual case, and what
  your log-capture/dashboard already are), wrap the bridge in a
  :class:`UdpTargetLink`. MuJoCo sends a small JSON datagram of target angles
  and gets the latest motor state back in the reply. See
  ``mujoco_stream_example.py`` for the MuJoCo end.

Frames and units
----------------
* Angles are **radians in the user frame** - the same convention the worker,
  the panels and ``sequence`` use. A MuJoCo hinge ``qpos`` is already radians,
  so no conversion is needed; if your MuJoCo export is in degrees, convert
  before calling (``math.radians``), exactly as ``jointlog`` does for the logs.
* A motor's **zero must line up with MuJoCo's zero** for that joint, or every
  command lands offset by the mismatch. Zero-calibrate the target motors to the
  sim's frame first (Set Zero at the sim's home pose), the same caveat the
  ``jointlog`` docstring gives for replay.

Safety
------
This bridge only *streams setpoints*; it never enables a motor implicitly. Bring
the motors up the normal way (enable + choose a mode in the dashboard, or call
:meth:`arm`) so safe-enable seeds the hold at the current shaft position. If a
streamed value is out of range the worker clamps it; if feedback leaves the
calibrated range the worker's cutout disables the motor. :meth:`estop` forwards
to the global E-STOP latch.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional

from PySide6.QtCore import QObject, Signal

from .. import worker as wk
from ..protocol import RunMode


@dataclass(frozen=True)
class JointState:
    """One motor's latest measured feedback, in the user frame (radians)."""

    device_id: int
    position: float          # rad
    velocity: float          # rad/s
    torque: float            # Nm
    temperature: float       # deg C
    t_monotonic: float       # time.monotonic() when this sample arrived

    def as_dict(self) -> dict:
        return {
            "id": self.device_id,
            "pos": self.position,
            "vel": self.velocity,
            "torque": self.torque,
            "temp": self.temperature,
            "t": self.t_monotonic,
        }


class MujocoBridge(QObject):
    """Streams sim joint angles onto motors and captures motor feedback.

    ``joint_map`` maps a MuJoCo joint *label* (any string you pick - the joint
    name, ``"revolute_3"``, etc.) to the motor's CAN id it drives. Only mapped
    joints are commanded, so a 6-DoF sim can drive however many motors are
    actually connected - the same "map channels to ids" idea as the sequencer's
    ``channel_map``.

    ``mode`` is the run-mode the streamed setpoints target:

    * ``RunMode.POSITION_PP`` / ``POSITION_CSP`` (default) - position tracking;
      only the angle is sent. This is the "make the real arm follow the sim
      pose" case.
    * ``RunMode.MIT`` - impedance/torque: the angle is sent as the MIT position
      target together with ``send_kp``/``send_kd`` gains and an optional
      per-call feed-forward torque, so you can forward the sim's own torques for
      a compliant follow. Set the mode on the motors (dashboard or :meth:`arm`)
      to match.

    ``max_rate_hz`` throttles outgoing posts: the worker only puts a frame on
    the wire at its own loop rate and always uses the *latest* target, so
    posting faster than that just fills the queue. Coalescing here keeps the
    queue shallow when MuJoCo steps at 500-1000 Hz.
    """

    #: Emitted whenever a fresh feedback sample arrives, for the UI/plots.
    stateUpdated = Signal(object)   # JointState

    def __init__(
        self,
        worker: "wk.ControlWorker",
        joint_map: dict[str, int],
        *,
        mode: int = RunMode.POSITION_PP,
        max_rate_hz: float = 250.0,
        send_kp: Optional[float] = None,
        send_kd: Optional[float] = None,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._worker = worker
        self._joint_map = dict(joint_map)
        self._mode = int(mode)
        self._send_kp = send_kp
        self._send_kd = send_kd
        self._min_interval = 1.0 / max_rate_hz if max_rate_hz > 0 else 0.0
        self._last_post = 0.0

        # Newest feedback per CAN id, guarded because statusUpdated may be
        # delivered on the worker thread while the UDP/receive side reads it.
        self._lock = threading.Lock()
        self._state: dict[int, JointState] = {}

        # reverse map: CAN id -> joint label, for state lookups by joint.
        self._id_to_joint = {cid: name for name, cid in self._joint_map.items()}

        worker.statusUpdated.connect(self._on_status)

    # -- configuration -----------------------------------------------------------

    @property
    def joint_map(self) -> dict[str, int]:
        return dict(self._joint_map)

    def set_joint_map(self, joint_map: dict[str, int]) -> None:
        """Replace which sim joints drive which motors.

        Lets a caller rebuild the mapping after motors connect (motors are
        usually added *after* the window opens) without recreating the bridge
        and re-subscribing to ``statusUpdated``. Feedback already cached for a
        CAN id that is still mapped stays valid; anything else falls out of
        :meth:`latest_state` because the reverse map no longer names it.
        """
        self._joint_map = dict(joint_map)
        self._id_to_joint = {cid: name for name, cid in self._joint_map.items()}

    def set_mode(self, mode: int) -> None:
        """Change which run-mode streamed setpoints target (does not touch the
        motors - set their mode too via :meth:`arm` or the dashboard)."""
        self._mode = int(mode)

    # -- sim -> motor ------------------------------------------------------------

    def push_targets(
        self,
        targets: dict[str, float],
        *,
        velocities: Optional[dict[str, float]] = None,
        torques_ff: Optional[dict[str, float]] = None,
    ) -> int:
        """Stream one frame of joint angles to the mapped motors.

        ``targets`` maps joint label -> angle (rad, user frame). Unmapped labels
        are ignored; non-finite values are skipped. ``velocities`` (rad/s) and
        ``torques_ff`` (Nm, MIT feed-forward only) are optional per-joint extras.
        Returns the number of motors actually commanded.

        Throttled to ``max_rate_hz``: a call inside the min interval is dropped
        (the worker still holds the previous target), so it is safe to call this
        every MuJoCo step.
        """
        now = time.monotonic()
        if self._min_interval and (now - self._last_post) < self._min_interval:
            return 0
        self._last_post = now

        sent = 0
        for label, angle in targets.items():
            device_id = self._joint_map.get(label)
            if device_id is None:
                continue
            if not (isinstance(angle, (int, float)) and math.isfinite(angle)):
                continue

            kwargs: dict = {"position": float(angle)}
            if self._mode == RunMode.MIT:
                if self._send_kp is not None:
                    kwargs["kp"] = float(self._send_kp)
                if self._send_kd is not None:
                    kwargs["kd"] = float(self._send_kd)
                if torques_ff is not None:
                    tq = torques_ff.get(label)
                    if tq is not None and math.isfinite(tq):
                        kwargs["torque_ff"] = float(tq)
            if velocities is not None:
                vel = velocities.get(label)
                if vel is not None and math.isfinite(vel):
                    kwargs["velocity"] = float(vel)

            self._worker.post(wk.SetTarget(device_id=device_id, **kwargs))
            sent += 1
        return sent

    # -- motor -> sim ------------------------------------------------------------

    def _on_status(self, device_id: int, status) -> None:
        """Slot for worker.statusUpdated: cache the newest feedback per motor."""
        if device_id not in self._id_to_joint:
            return
        sample = JointState(
            device_id=device_id,
            position=float(status.position),
            velocity=float(status.velocity),
            torque=float(status.torque),
            temperature=float(status.temperature),
            t_monotonic=time.monotonic(),
        )
        with self._lock:
            self._state[device_id] = sample
        self.stateUpdated.emit(sample)

    def latest_state(self) -> dict[str, JointState]:
        """Newest measured feedback keyed by joint label. A joint that has not
        reported yet is simply absent."""
        with self._lock:
            snapshot = dict(self._state)
        return {self._id_to_joint[cid]: s for cid, s in snapshot.items()
                if cid in self._id_to_joint}

    def state_payload(self) -> dict:
        """JSON-friendly ``{joint_label: {...}}`` snapshot for the UDP reply."""
        return {label: s.as_dict() for label, s in self.latest_state().items()}

    # -- motor bring-up helpers (optional, explicit) -----------------------------

    def arm(self, mode: Optional[int] = None) -> None:
        """Set the run-mode on every mapped motor and enable it.

        Convenience only, and deliberately explicit - it energises real motors.
        The worker's safe-enable seeds the hold at each shaft's current position,
        so an armed motor holds where it is until the first :meth:`push_targets`.
        Prefer arming the motors from the dashboard when you can, so an operator
        is watching. Set the motors' mode to match the mode you stream in.
        """
        target_mode = self._mode if mode is None else int(mode)
        self._mode = target_mode
        for device_id in self._joint_map.values():
            self._worker.post(wk.SetMode(device_id=device_id, mode=target_mode))
            self._worker.post(wk.Enable(device_id=device_id))

    def disarm(self) -> None:
        """Disable every mapped motor (leaves the link up)."""
        for device_id in self._joint_map.values():
            self._worker.post(wk.Disable(device_id=device_id))

    def estop(self, engage: bool = True) -> None:
        """Forward to the global E-STOP latch (disables all motors when engaged)."""
        self._worker.post(wk.EStop(engage=engage))


class UdpTargetLink:
    """UDP front-end so an out-of-process MuJoCo script can drive the bridge.

    Runs a background receive loop. Each datagram is a JSON object of target
    angles; the loop pushes it to the wrapped :class:`MujocoBridge` and replies
    to the sender with the latest motor state, so MuJoCo gets feedback in the
    same round-trip. Request/reply keeps it stateless - no separate feedback
    socket, no port bookkeeping on the sim side.

    Wire format (request, from MuJoCo)::

        {"targets": {"revolute_1": 0.12, "revolute_2": -0.4, ...},
         "velocities": {"revolute_1": 0.0, ...},   # optional
         "torques_ff": {"revolute_1": 0.3, ...}}   # optional (MIT only)

    Reply (to MuJoCo)::

        {"state": {"revolute_1": {"id": 1, "pos": 0.118, "vel": 0.01,
                                   "torque": 0.22, "temp": 34.0, "t": 1234.5},
                    ...}}

    This speaks only to the bridge's public API, so it stays a thin adapter.
    Bind to ``127.0.0.1`` when the sim and GUI share a machine; bind to
    ``0.0.0.0`` to let a sim on another host reach the machine with the CAN
    adapter.
    """

    def __init__(self, bridge: MujocoBridge, host: str = "127.0.0.1",
                 port: int = 8642, recv_bufsize: int = 65535) -> None:
        self._bridge = bridge
        self._host = host
        self._port = port
        self._bufsize = recv_bufsize
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self) -> None:
        if self._running:
            return
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.settimeout(0.5)   # so the loop can notice stop()
        self._running = True
        self._thread = threading.Thread(
            target=self._serve, name="mujoco-udp-link", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    @property
    def address(self) -> tuple[str, int]:
        return (self._host, self._port)

    def _serve(self) -> None:
        assert self._sock is not None
        while self._running:
            try:
                data, addr = self._sock.recvfrom(self._bufsize)
            except socket.timeout:
                continue
            except OSError:
                break   # socket closed under us during stop()
            try:
                msg = json.loads(data.decode("utf-8"))
                targets = msg.get("targets") or {}
                velocities = msg.get("velocities")
                torques_ff = msg.get("torques_ff")
                if isinstance(targets, dict):
                    self._bridge.push_targets(
                        {str(k): float(v) for k, v in targets.items()},
                        velocities=_num_map(velocities),
                        torques_ff=_num_map(torques_ff),
                    )
                reply = json.dumps({"state": self._bridge.state_payload()})
                self._sock.sendto(reply.encode("utf-8"), addr)
            except (ValueError, TypeError, KeyError):
                # A malformed datagram must not kill the link; drop it and keep
                # serving. (An attacker on the socket is out of scope - bind to
                # localhost unless you trust the network.)
                continue


def _num_map(obj) -> Optional[dict[str, float]]:
    """Coerce an optional ``{label: number}`` JSON object to floats, or None."""
    if not isinstance(obj, dict):
        return None
    out: dict[str, float] = {}
    for k, v in obj.items():
        try:
            out[str(k)] = float(v)
        except (TypeError, ValueError):
            continue
    return out or None