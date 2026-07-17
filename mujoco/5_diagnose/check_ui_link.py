"""Diagnose the GUI's MuJoCo UDP link (port 8642) before running the sim scripts.

``mujoco_teach.py`` / ``mujoco_dashboard.py`` talk to the RobStride GUI over a
UDP request/reply on ``127.0.0.1:8642`` (the SimBridge dock). When teach prints
"no motors reporting", three different faults look identical from the script:

  1. the link is not listening   -> no reply at all (dock not toggled on / GUI
     not running / wrong host-port),
  2. the link is up but no motors are mapped/reporting  -> a reply arrives but
     its ``state`` is empty (no motors connected, or none enabled so no status
     has flowed yet), or
  3. motors report under names that don't match the model's joints.

This tool sends one no-command poll and prints the raw reply so you can tell
which case you are in. Run it, watching the ``pos`` values change as you move a
limp motor by hand::

    python mujoco/5_diagnose/check_ui_link.py            # single snapshot
    python mujoco/5_diagnose/check_ui_link.py --watch    # poll ~5 Hz until Ctrl-C
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time

HOST, PORT = "127.0.0.1", 8642
_TIMEOUT_S = 0.5


def poll_once(sock: socket.socket) -> dict | None:
    """Send a no-command poll; return the parsed ``state`` dict, or None on no/bad
    reply. An empty dict (``{}``) means the link answered but reported no motors."""
    try:
        sock.sendto(json.dumps({"targets": {}}).encode("utf-8"), (HOST, PORT))
        data, _ = sock.recvfrom(65535)
    except socket.timeout:
        return None
    except OSError as exc:
        print(f"socket error: {exc}", file=sys.stderr)
        return None
    try:
        msg = json.loads(data.decode("utf-8"))
    except ValueError:
        print(f"reply was not JSON: {data!r}", file=sys.stderr)
        return None
    state = msg.get("state")
    return state if isinstance(state, dict) else {}


def _describe(state: dict | None) -> str:
    if state is None:
        return ("NO REPLY - the link is not listening on "
                f"{HOST}:{PORT}. Start the GUI and toggle the SimBridge dock ON.")
    if not state:
        return ("link is UP but reports NO motors - connect the motors in the "
                "GUI, enable them (LIMP for hand-teaching), then re-toggle the "
                "SimBridge dock so the joint map is rebuilt.")
    lines = [f"link UP, {len(state)} motor(s) reporting:"]
    for name in sorted(state):
        s = state[name]
        lines.append(
            f"  {name:<14} id={s.get('id')} "
            f"pos={s.get('pos', float('nan')):+.4f} rad "
            f"vel={s.get('vel', float('nan')):+.4f} "
            f"torque={s.get('torque', float('nan')):+.3f} Nm")
    return "\n".join(lines)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--watch", action="store_true",
                   help="poll continuously (~5 Hz) until Ctrl-C")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(_TIMEOUT_S)
    try:
        if not args.watch:
            print(_describe(poll_once(sock)))
            return
        print(f"watching {HOST}:{PORT} - move a limp motor by hand; Ctrl-C to stop")
        while True:
            state = poll_once(sock)
            print("\n" + _describe(state))
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
