"""Simulation bridge: stream MuJoCo joint angles onto the motors and read back.

The GUI owns the *motor* side of the bridge (:class:`MujocoBridge` +
:class:`UdpTargetLink`), started from the Simulation dock. An out-of-process
MuJoCo script (``mujoco/3_teleop/mujoco_dashboard.py`` /
``mujoco/4_teach_replay/mujoco_teach.py``) speaks to it over UDP on
``127.0.0.1:8642``. See ``mujoco_bridge`` for the wire format and safety notes,
and ``mujoco/README.md`` for the end-to-end walkthrough.
"""

from __future__ import annotations

from .mujoco_bridge import JointState, MujocoBridge, UdpTargetLink

__all__ = ["JointState", "MujocoBridge", "UdpTargetLink"]
