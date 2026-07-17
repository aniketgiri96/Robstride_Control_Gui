# MuJoCo simulation & teleop — start here

This folder holds everything that connects the **MuJoCo physics simulator** to the
**real RobStride motors**. The files are grouped into numbered stages that match
the order you actually use them. Read top-to-bottom once, then jump to the stage
you need.

## The big picture (read this first)

There are always **two processes**:

```
  ┌─────────────────────────┐        UDP 127.0.0.1:8642        ┌──────────────────────────┐
  │  MuJoCo script          │  targets (joint angles) ───────▶ │  RobStride GUI            │
  │  (this folder)          │                                  │  (main.py)                │
  │  dashboard / teach      │ ◀─────── motor state (feedback)  │  Simulation dock  ◀──▶ CAN │
  └─────────────────────────┘                                  └──────────────────────────┘
```

- The **GUI owns the motors.** Its code lives in the package, not here:
  - `robstride_gui/sim/mujoco_bridge.py` — the bridge (`MujocoBridge` streams sim
    angles onto motors; `UdpTargetLink` is the UDP server on port 8642).
  - `robstride_gui/ui/sim_dock.py` — the **Simulation dock**, the on/off switch for
    the bridge inside the GUI.
- The **MuJoCo scripts in this folder are separate programs.** They send joint
  angles to the GUI and read motor feedback back — over UDP, nothing more. They
  never touch the CAN bus directly.

Because they talk over a socket, you start the GUI first, tick the **Simulation**
dock ON, then run a script from this folder.

> **Safety:** the bridge only *streams setpoints*. It never enables a motor. You
> always bring motors up (enable + pick a mode) in the GUI, where an operator is
> watching. An out-of-range value is clamped by the GUI, and the E-STOP latch
> overrides everything.

## Folder map (the feature sequence)

| Stage | Folder | What it does | You run |
|------:|--------|--------------|---------|
| — | `requirements-sim.txt` | Extra deps for these scripts (mujoco, numpy, matplotlib). The GUI itself does **not** need them. | `pip install -r mujoco/requirements-sim.txt` |
| 1 | `1_build_scene/` | Turn the robot's URDF + STL meshes into a MuJoCo `scene.xml`. | `urdf_to_scene.py` |
| 2 | `2_run_bridge/` | Config that pins each MuJoCo joint name to a motor CAN id (`sim_bindings.json`). The bridge **code** is in `robstride_gui/`. | (edited, read by the GUI) |
| 3 | `3_teleop/` | Live dashboard: sliders + move-sequences + plots, optionally streaming to the real motors. | `mujoco_dashboard.py` |
| 4 | `4_teach_replay/` | Teach-by-hand: record the robot as you move it by hand, replay it in the viewer. Example recordings under `recordings/`. | `mujoco_teach.py` |
| 5 | `5_diagnose/` | One-shot check that the GUI's UDP link is up and motors are reporting. Run this when something looks wrong. | `check_ui_link.py` |

**Run every command from the repo root** (the folder above `mujoco/`) so the
`robstride_gui` package resolves and the relative paths below line up.

---

## Stage 1 — Build the scene  (`1_build_scene/`)

MuJoCo needs a model file. `urdf_to_scene.py` converts the Onshape URDF export
(`main_assembly.urdf` + the `main_assembly/` meshes) into a MuJoCo `scene.xml`,
adding a floor, a light, and one position actuator per revolute joint. The hinge
joints keep their URDF names (`revolute_1`..`revolute_6`) so they line up with the
motor map in Stage 2.

```bash
python mujoco/1_build_scene/urdf_to_scene.py \
    mujoco/1_build_scene/main_assembly.urdf \
    mujoco/1_build_scene/scene.xml
```

`scene.xml` is already committed, so you only rerun this if the robot geometry
changes. Note `scene.xml` includes `main_assembly/urdf/main_assembly.xml` by a
**relative** path — keep `scene.xml` and the `main_assembly/` folder together.

## Stage 2 — Wire joints to motors  (`2_run_bridge/`)

`sim_bindings.json` maps each MuJoCo **joint name** to a motor **CAN id**:

```json
{ "revolute_1": 1, "revolute_2": 2, "revolute_3": 3,
  "revolute_4": 4, "revolute_5": 5, "revolute_6": 6 }
```

The GUI reads this when you tick the Simulation dock ON, and only keeps ids that
are actually connected. If the file is missing it falls back to
`revolute_<id> -> <id>`. The bridge itself (`MujocoBridge`, `UdpTargetLink`) lives
in `robstride_gui/sim/` — this folder only holds its config.

**To run the bridge:** launch the GUI (`python main.py`), connect + enable your
motors, then tick **Simulation → "Stream MuJoCo bridge (UDP 127.0.0.1:8642)"**.

## Stage 3 — Teleop dashboard  (`3_teleop/`)

`mujoco_dashboard.py` opens the MuJoCo viewer plus a Tk control panel: per-joint
sliders, a recordable move-sequence, and live position/velocity/torque plots.
Tick **"Stream to real motor (UDP)"** to also drive the physical arm (off by
default so nothing jumps on launch); the dashed plot lines overlay the motors'
measured positions.

```bash
python mujoco/3_teleop/mujoco_dashboard.py mujoco/1_build_scene/scene.xml
# (no argument also works — it defaults to ../1_build_scene/scene.xml)
```

## Stage 4 — Teach by hand & replay  (`4_teach_replay/`)

`mujoco_teach.py` records the robot's motion **while you move it by hand** and
replays it. To record, the motors must be **LIMP** in the GUI (MIT mode with
kp=0, kd=0, then Enable — or the calibration "make limp"), so they report their
encoder angle while free to move.

```bash
# record (move by hand, Ctrl-C to stop)
python mujoco/4_teach_replay/mujoco_teach.py \
    mujoco/1_build_scene/scene.xml \
    --record mujoco/4_teach_replay/recordings/demo1.csv

# replay a recording in the viewer (sim only, no motors), looping
python mujoco/4_teach_replay/mujoco_teach.py \
    mujoco/1_build_scene/scene.xml \
    --replay mujoco/4_teach_replay/recordings/demo1.csv --loop
```

Recordings are CSV in the same column layout as the dashboard's log, so they can
also be played back onto real motors via the GUI's sequencer. Replay is smoothed
by `robstride_gui/interp.py` (the raw ~5 Hz capture is a staircase; add `--raw` to
skip smoothing). Example recordings live in `recordings/`.

## Stage 5 — Diagnose the link  (`5_diagnose/`)

If a script prints "no motors reporting", run this to tell *which* of three
faults you have (link not listening / no motors mapped / name mismatch):

```bash
python mujoco/5_diagnose/check_ui_link.py           # single snapshot
python mujoco/5_diagnose/check_ui_link.py --watch   # ~5 Hz, watch pos change as you move a limp motor
```

---

## Typical first run, end to end

```bash
pip install -r requirements.txt -r mujoco/requirements-sim.txt   # once
python main.py                                                   # 1. start the GUI
#   → connect motors, enable them, tick the "Simulation" dock ON
python mujoco/5_diagnose/check_ui_link.py                        # 2. confirm the link is up
python mujoco/3_teleop/mujoco_dashboard.py                       # 3. drive it
```

## Where the GUI-side code lives (not in this folder)

These stay in the package because the GUI imports them:

- `robstride_gui/sim/mujoco_bridge.py` — `MujocoBridge` + `UdpTargetLink` + wire format
- `robstride_gui/ui/sim_dock.py` — the Simulation dock (on/off switch)
- `robstride_gui/interp.py` — replay smoothing used by Stage 4
