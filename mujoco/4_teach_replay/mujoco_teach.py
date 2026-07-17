"""Teach by hand: record the robot's hand-moved motion, replay it in MuJoCo.

Two modes:

  --record FILE : the motors are enabled LIMP in the GUI so you can move them by
                  hand. This polls their measured positions, shows them live in
                  the MuJoCo viewer, and logs every frame to FILE.
  --replay FILE : plays a recorded FILE back in the MuJoCo viewer (sim only, no
                  motors). Add --loop to repeat.

Prerequisite for --record
--------------------------
In the RobStride GUI, enable every motor you want to record in a LIMP state, so
it reports its encoder position while free to move by hand (a held Position-mode
motor resists your hand and records nothing):

  * MIT mode with kp = 0 and kd = 0, then Enable, or
  * the GUI's range-calibration "make limp" (the same limp used for calibration).

The GUI's UdpTargetLink must be running (port 8642). Verify with
mujoco/5_diagnose/check_ui_link.py: the motors should report, and the pos values
should change as you move them.

The log is written in the same column layout as the dashboard's joint_log.csv
(time + cmd/pos/vel/torque per joint, degrees), so a recording of joints named
revolute_1..N can also be replayed onto the real motors later via the RobStride
GUI's sequencer / jointlog, not only here.

Run (from the repo root, so the ``robstride_gui`` package resolves)
------------------------------------------------------------------
    # move by hand, Ctrl-C to stop
    python mujoco/4_teach_replay/mujoco_teach.py \
        mujoco/1_build_scene/scene.xml \
        --record mujoco/4_teach_replay/recordings/demo1.csv
    # replay a recording, looping
    python mujoco/4_teach_replay/mujoco_teach.py \
        mujoco/1_build_scene/scene.xml \
        --replay mujoco/4_teach_replay/recordings/demo1.csv --loop
"""

import argparse
import csv
import json
import math
import os
import socket
import sys
import time

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    sys.exit("mujoco is not installed - run: pip install mujoco")

# This script lives at mujoco/4_teach_replay/; add the repo root (three levels up)
# to sys.path so ``robstride_gui`` imports whether it is launched from the repo
# root or from inside the mujoco/ folder.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from robstride_gui.interp import smooth_columns

GUI_HOST, GUI_PORT = "127.0.0.1", 8642


def joint_qpos_map(model) -> dict:
    """Every named joint -> (qpos_index, dof_index)."""
    out = {}
    for jid in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if name is not None:
            out[name] = (int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid]))
    return out


# -- UDP polling (read motor state without commanding anything) ------------------

def _send_poll(sock) -> None:
    """Send an empty (no-command) datagram so the GUI replies with motor state."""
    try:
        sock.sendto(json.dumps({"targets": {}}).encode("utf-8"), (GUI_HOST, GUI_PORT))
    except OSError:
        pass


def _drain(sock):
    """Return the newest state dict waiting in the socket, or None (non-blocking)."""
    latest = None
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            st = json.loads(data.decode("utf-8")).get("state")
            if isinstance(st, dict):
                latest = st
        except (BlockingIOError, OSError):
            break
        except ValueError:
            break
    return latest


# -- record ----------------------------------------------------------------------

def _open_log(path: str, joints):
    """Open the CSV and write the dashboard-layout header; return (file, writer)."""
    f = open(path, "w", newline="")
    w = csv.writer(f)
    header = ["time"]
    for j in joints:
        header += [f"cmd_{j}_deg", f"pos_{j}_deg", f"vel_{j}_degps", f"torque_{j}_Nm"]
    w.writerow(header)
    return f, w


def record(model, data, path: str, rate: float) -> None:
    addr = joint_qpos_map(model)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    # Open the viewer FIRST so there is always a window, even before any motor
    # reports. Discovery and logging then start lazily, the moment the GUI first
    # reports a limp motor whose name matches a model joint. This way "no motors
    # yet" shows an idle scene instead of exiting with no window at all.
    print("viewer open - waiting for the GUI to report limp motors ...")
    print("(enable motors LIMP in the GUI: MIT kp=0 kd=0, or calibration make-limp)")

    joints = []          # resolved on first matching report
    f = w = None         # log opened lazily once joints are known
    period = 1.0 / rate if rate > 0 else 0.0
    t0 = None
    frames = 0
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                tick = time.monotonic()
                _send_poll(sock)
                state = _drain(sock)
                if state:
                    if not joints:
                        # First report in: figure out which joints we can record.
                        joints = [j for j in state if j in addr]
                        if joints:
                            f, w = _open_log(path, joints)
                            t0 = tick
                            print("recording joints:", ", ".join(joints))
                            print("move the robot by hand; press Ctrl-C "
                                  "(or close the viewer) to stop")

                    # Place the measured pose into the sim so the viewer mirrors
                    # your hand movement in real time.
                    for j in joints:
                        s = state.get(j)
                        if not s:
                            continue
                        qa, da = addr[j]
                        data.qpos[qa] = float(s.get("pos", data.qpos[qa]))
                        data.qvel[da] = float(s.get("vel", 0.0))
                    mujoco.mj_forward(model, data)

                    if w is not None:
                        row = [round(tick - t0, 4)]
                        for j in joints:
                            s = state.get(j, {})
                            pos_deg = math.degrees(float(s.get("pos", 0.0)))
                            vel_deg = math.degrees(float(s.get("vel", 0.0)))
                            torque = float(s.get("torque", 0.0))
                            # cmd == pos here: the hand IS the command.
                            row += [round(pos_deg, 3), round(pos_deg, 3),
                                    round(vel_deg, 3), round(torque, 3)]
                        w.writerow(row)
                        frames += 1

                viewer.sync()
                if period:
                    dt = time.monotonic() - tick
                    if dt < period:
                        time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        if f is not None:
            f.close()
        sock.close()
    if frames:
        print(f"\nsaved {frames} frames to {path}")
    else:
        print("\nno motors reported while the viewer was open - nothing recorded.\n"
              "Enable the motors LIMP in the GUI and re-toggle the SimBridge dock, "
              "then run again.")


# -- replay ----------------------------------------------------------------------

def replay(model, data, path: str, loop: bool, smooth: bool = True) -> None:
    addr = joint_qpos_map(model)
    with open(path) as fh:
        rows = list(csv.reader(fh))
    # Skip leading blank rows (a stray CRLF before the header would otherwise be
    # read as an empty header with no columns).
    start = next((i for i, r in enumerate(rows)
                  if any(cell.strip() for cell in r)), len(rows))
    rows = rows[start:]
    if len(rows) < 2:
        sys.exit(f"{path} has no data rows")
    cols = [c.strip() for c in rows[0]]
    data_rows = [r for r in rows[1:] if any(cell.strip() for cell in r)]

    joints = [c[len("pos_"):-len("_deg")] for c in cols
              if c.startswith("pos_") and c.endswith("_deg")
              and c[len("pos_"):-len("_deg")] in addr]
    if not joints:
        sys.exit("no matching pos_<joint>_deg columns for this model")
    if "time" not in cols:
        sys.exit("recording has no 'time' column")
    tcol = cols.index("time")
    pcol = {j: cols.index(f"pos_{j}_deg") for j in joints}

    # Parse into a timeline + one radian column per joint so the coarse
    # zero-order-hold capture can be smoothed into continuous motion before it is
    # played. Without smoothing the viewer reproduces the recorded staircase:
    # each joint holds flat for ~200 ms then jumps several degrees in one frame.
    times = [float(r[tcol]) for r in data_rows]
    tracks = {j: [math.radians(float(r[pcol[j]])) for r in data_rows] for j in joints}
    if smooth:
        smoothed = smooth_columns(times, [tracks[j] for j in joints])
        tracks = {j: smoothed[i] for i, j in enumerate(joints)}
    print("replaying joints:", ", ".join(joints),
          "(smoothed)" if smooth else "(raw)")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            wall0 = time.monotonic()
            base = times[0]
            for frame, t in enumerate(times):
                if not viewer.is_running():
                    break
                for j in joints:
                    qa, _ = addr[j]
                    data.qpos[qa] = tracks[j][frame]
                mujoco.mj_forward(model, data)
                viewer.sync()
                # Pace playback to the recorded timestamps.
                target = t - base
                while viewer.is_running() and (time.monotonic() - wall0) < target:
                    time.sleep(0.001)
            if not loop:
                break
    print("done")


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("model", help="path to the MuJoCo model XML")
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--record", metavar="FILE", help="record hand motion to FILE")
    mode.add_argument("--replay", metavar="FILE", help="replay FILE in the viewer")
    p.add_argument("--rate", type=float, default=100.0, help="record rate Hz (default 100)")
    p.add_argument("--loop", action="store_true", help="loop the replay")
    p.add_argument("--raw", action="store_true",
                   help="replay recorded samples verbatim (skip motion smoothing)")
    args = p.parse_args()

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    if args.record:
        record(model, data, args.record, args.rate)
    else:
        replay(model, data, args.replay, args.loop, smooth=not args.raw)


if __name__ == "__main__":
    main()