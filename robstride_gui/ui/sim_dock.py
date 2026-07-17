"""Simulation dock: start/stop the MuJoCo <-> motor UDP bridge.

The heavy lifting lives in :mod:`robstride_gui.sim.mujoco_bridge`; this dock is
only its on/off switch and status readout. It owns one :class:`MujocoBridge`
(subscribed to the worker's feedback for the window's lifetime) and a
:class:`UdpTargetLink` that it starts/stops with the checkbox.

An out-of-process MuJoCo script (``mujoco/3_teleop/mujoco_dashboard.py`` /
``mujoco/4_teach_replay/mujoco_teach.py``)
sends target angles keyed by *joint name*; the bridge maps those names to CAN
ids. The map is rebuilt from the currently connected motors every time the link
is (re)started, and again via :meth:`SimBridgeDock.refresh_joint_map` whenever a
motor is added or removed while it streams, so motors that connect after the
bridge starts are picked up automatically - no off/on toggle needed.

Safety: starting the link only opens a socket and lets the sim *stream
setpoints*. It never enables a motor. Bring motors up the normal way (enable +
mode in the dashboard); a streamed value is clamped by the worker's safety
limits, and the E-STOP latch still overrides everything.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from PySide6.QtWidgets import (
    QCheckBox, QDockWidget, QLabel, QVBoxLayout, QWidget,
)

from ..sim import MujocoBridge, UdpTargetLink

#: Where the GUI listens for the MuJoCo scripts. Localhost by default; the two
#: standalone scripts hard-code the same host/port.
BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 8642

#: Min seconds between feedback-driven label refreshes (feedback arrives at the
#: worker's poll rate; the label does not need to).
_STATUS_MIN_INTERVAL_S = 0.2


class SimBridgeDock(QDockWidget):
    """Dockable on/off switch + status for the MuJoCo UDP bridge.

    ``joint_map_provider`` is called each time the link is started and must
    return a ``{joint_name: can_id}`` map for the currently connected motors.
    """

    def __init__(
        self,
        worker,
        joint_map_provider: Callable[[], dict[str, int]],
        *,
        host: str = BRIDGE_HOST,
        port: int = BRIDGE_PORT,
        parent=None,
    ) -> None:
        super().__init__("Simulation", parent)
        self._worker = worker
        self._provider = joint_map_provider
        self._host = host
        self._port = port

        self._bridge: Optional[MujocoBridge] = None
        self._link: Optional[UdpTargetLink] = None
        self._last_status = 0.0

        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(6, 6, 6, 6)

        self._check = QCheckBox(f"Stream MuJoCo bridge (UDP {host}:{port})")
        self._check.setToolTip(
            "Open the UDP link so a MuJoCo script can drive the motors and read "
            "their feedback. Does NOT enable motors - bring them up in the "
            "dashboard first. The map (joint name -> CAN id) is rebuilt from the "
            "connected motors each time you tick this.")
        self._check.toggled.connect(self._on_toggled)
        lay.addWidget(self._check)

        self._status = QLabel("bridge: off")
        self._status.setStyleSheet("color:#888;")
        self._status.setWordWrap(True)
        lay.addWidget(self._status)
        lay.addStretch(1)

        self.setWidget(w)

    # -- lifecycle ---------------------------------------------------------------

    def _ensure_bridge(self) -> MujocoBridge:
        """Create the bridge once, on first use (it subscribes to worker
        feedback for the window's lifetime)."""
        if self._bridge is None:
            self._bridge = MujocoBridge(self._worker, {})
            self._bridge.stateUpdated.connect(self._on_state)
        return self._bridge

    def _on_toggled(self, checked: bool) -> None:
        if checked:
            self._start()
        else:
            self._stop()

    def _start(self) -> None:
        try:
            joint_map = dict(self._provider())
        except Exception as exc:  # provider is caller-supplied; never crash the UI
            joint_map = {}
            self._status.setText(f"bridge: could not build joint map ({exc})")
        bridge = self._ensure_bridge()
        bridge.set_joint_map(joint_map)

        if self._link is None:
            self._link = UdpTargetLink(bridge, host=self._host, port=self._port)
        try:
            self._link.start()
        except OSError as exc:
            # Port busy / bind failure: don't leave the checkbox looking "on".
            self._link = None
            self._check.blockSignals(True)
            self._check.setChecked(False)
            self._check.blockSignals(False)
            self._status.setText(
                f"bridge: could not listen on {self._host}:{self._port} ({exc})")
            return

        self._set_mapped_status(joint_map)

    def _set_mapped_status(self, joint_map: dict[str, int]) -> None:
        """Render the 'listening / N joints mapped' status for a joint map."""
        n = len(joint_map)
        hint = "" if n else "  (no motors connected - connect them, then re-tick)"
        mapped = ", ".join(f"{name}->{cid}" for name, cid in sorted(joint_map.items()))
        self._status.setText(
            f"bridge: listening on {self._host}:{self._port} - "
            f"{n} joint(s) mapped{hint}\n{mapped}")

    def refresh_joint_map(self) -> None:
        """Rebuild the joint map from the currently connected motors, live.

        Called by the window when a motor is added or removed so a motor that
        connects *after* the link was started is picked up without an off/on
        toggle - the exact footgun of the toggle-time-only snapshot. A no-op while
        the link is stopped (the next :meth:`_start` rebuilds the map anyway).
        """
        if self._link is None or self._bridge is None:
            return
        try:
            joint_map = dict(self._provider())
        except Exception as exc:  # provider is caller-supplied; never crash the UI
            self._status.setText(f"bridge: could not rebuild joint map ({exc})")
            return
        self._bridge.set_joint_map(joint_map)
        self._set_mapped_status(joint_map)

    def _stop(self) -> None:
        if self._link is not None:
            self._link.stop()
        self._status.setText("bridge: off")

    def _on_state(self, sample) -> None:
        """Show the newest feedback, throttled - the bridge relays every poll."""
        if not self._check.isChecked():
            return
        now = time.monotonic()
        if (now - self._last_status) < _STATUS_MIN_INTERVAL_S:
            return
        self._last_status = now
        self._status.setText(
            f"bridge: listening on {self._host}:{self._port} - "
            f"last: motor {sample.device_id} = {sample.position:+.3f} rad")

    def shutdown(self) -> None:
        """Stop the link cleanly (call from the window's closeEvent)."""
        if self._link is not None:
            self._link.stop()
            self._link = None
