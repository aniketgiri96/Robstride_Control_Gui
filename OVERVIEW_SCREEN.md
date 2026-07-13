# Overview Screen — How It Actually Works

The **Overview** tab is a single-screen control surface for all connected
RobStride motors: every motor's live status on one page, plus per-motor
calibration and motion controls that stay collapsed until you need them. It sits
as **tab 0** in the main window; the existing per-motor detail tabs are untouched
and remain available for deep work (MIT gains, sweep, device tools).

This document explains what each part does and how the data actually flows.

---

## 1. The big idea: it reuses the existing control loop

The Overview screen adds **no new hardware/IO code**. It does not touch the CAN
bus, serial port, or protocol. It is pure UI that talks to the same background
`ControlWorker` the rest of the app already uses, through the same two channels:

```
  UI  ──(post Command objects)──▶  ControlWorker (QThread)  ──▶  CAN bus
  UI  ◀──(Qt signals: status/power/…)──  ControlWorker
```

* **Commands out** — `SetTarget`, `SetMode`, `Enable`/`Disable`,
  `SetRangeLimits`. These are the exact commands the detail tabs post.
* **Signals in** — `statusUpdated`, `powerUpdated`, `motorEnabledChanged`,
  `rangeLimitsChanged`. The dashboard just renders them.

`MainWindow` is the single wiring hub. It fans each worker signal to *both* the
detail panel and the dashboard, and routes dashboard control signals to the same
handlers the panels use. So both surfaces drive a motor identically.

> **Why this matters:** the proven, safety-reviewed control path (`worker.py`,
> `bus.py`, `protocol.py`, `safety.py`) was not modified at all. All safety
> backstops below live in the worker and apply no matter which screen issued the
> command.

---

## 2. Files and responsibilities

| File | Role |
|---|---|
| `robstride_gui/ui/dashboard.py` | `MotorDashboard` — the screen: totals bar, the column of motor rows, and the sequence transport. Re-emits each row's signals upward. |
| `robstride_gui/ui/motor_row.py` | `MotorRow` — one motor's compact card: live readout, warning border, and the collapsible Setup (calibration + position controls). |
| `robstride_gui/ui/position_bar.py` | `PositionBar` — the min/max angle bar with commanded (ghost) and actual (solid) markers and a direction arrow. |
| `robstride_gui/ui/sequencer.py` | `SequencePlayer` — plays an imported animation frame-by-frame onto motors. |
| `robstride_gui/sequence.py` | Pure parser/model for imported sequences (JSON + CSV). No Qt. |
| `robstride_gui/calibration_store.py` | Persists per-motor calibration incl. the new `locked` flag. |
| `robstride_gui/ui/main_window.py` | Adds the dashboard as tab 0 and wires it to the worker. |

---

## 3. Layout

```
┌────────────────────────────────────────────────────────────────────┐
│ Σ current: 9.00 A   Σ torque: 6.00 Nm   active: 3   cycling: 1      │  ← totals bar (always on top)
├────────────────────────────────────────────────────────────────────┤
│ Sequence  [Import…]  12 frames × 6 channels @ 30 fps (0.4s)  ▶ ⏸ ⏹ │  ← sequence transport
├────────────────────────────────────────────────────────────────────┤
│ ● Motor 1  I:+1.5A  τ:+1.0Nm  ∠ +12.0°→+11.8° →  [═══◉═══]  Enable Setup ▸ │
│ ● Motor 2  I:+1.5A  τ:+1.0Nm  ∠ +0.0°→+0.0°      [══◉════]  Enable Setup ▸ │
│ … one compact row per motor (six fit without scrolling) …          │
└────────────────────────────────────────────────────────────────────┘
```

* The **totals bar** and **sequence bar** are fixed above the rows, so they stay
  visible regardless of scroll position.
* Rows live in a scroll area that only scrolls once a row's **Setup** is
  expanded — collapsed, all six read at a glance.

---

## 4. Per-motor live display (always visible)

Each row's top line updates at the worker's poll rate:

| Element | Source | Notes |
|---|---|---|
| State dot | `motorEnabledChanged` | green = enabled, red = disabled |
| Current `I` | `powerUpdated.iq` (A) | signed q-axis current |
| Torque `τ` | `statusUpdated.torque` (Nm) | |
| Angle `∠ cmd → act` | commanded (UI) → `statusUpdated.position` | see §7 |
| Direction arrow | sign of `statusUpdated.velocity` | `→`/`←`, hidden below a small dead-band |
| Position bar | actual + commanded markers between min/max | |
| Warning border | current/torque vs safety limit | red border at ≥ 80% of `SafetyLimits.for_model()` current/torque max |

Angles are shown in **degrees**; on the wire everything is **radians**
(converted only at the UI boundary).

---

## 5. Calibration (in the collapsed Setup)

CV1/CV2 are the motor's **min/max travel limits** — the same "range limits" the
worker already enforces.

Workflow:

1. **Unlock** — calibration starts **locked** so limits can't be nudged during
   operation. Click 🔒 → 🔓 to edit.
2. **Enter or capture** — type Min/Max in degrees, or click **Capture → Min** /
   **Capture → Max** to copy the motor's *current actual angle* into that field
   (avoids typing raw numbers).
3. **Save limits** — nothing is applied until you click Save. This emits
   `rangeLimitsEdited` → `SetRangeLimits`, so a half-edited value never reaches
   the motor.

Constraints:

* Min/Max spin boxes are hard-capped to the model's absolute travel
  (`model_limits()["position"]`), so the UI **cannot exceed the hardware range**.
* Saving also re-ranges the **Target / A / B** spins to the calibrated window, so
  you can't dial a motion command outside the safe travel either.
* The lock state is **persisted** per motor (`calibrations.json`, `locked`
  field) and restored on the next launch.

---

## 6. Position entry & sequencing (in the collapsed Setup + sequence bar)

**Manual target** — type an angle, click **Send**. Emits
`targetChanged {position}` → `SetTarget`.

**A/B jog** — set two angles A and B plus an interval, click **Start A/B**. A
per-row timer flips the target between A and B every interval. It first asserts
position mode, and **refuses to start if the motor is disabled**.

**Sequence** (screen-level, top bar):

1. **Import…** a Blender/animation export (JSON or CSV). The metadata line shows
   `N frames × M channels @ fps` immediately, so you see what you're about to
   send.
2. Channels map to motors by ascending CAN id (channel 0 → lowest id, …).
3. **▶ Play** advances one frame per tick at the sequence's fps, posting each
   channel's angle as a position target. It **refuses to play if no mapped motor
   is enabled**, and asserts position mode on the ones that will move.
4. **⏸ Stop** halts but keeps the frame (Play resumes). **⏹ Abort** halts and
   rewinds to the start.

### Sequence file formats

**JSON** (angles radians by default; `"units": "deg"` to override):

```json
{
  "fps": 30,
  "units": "rad",
  "channels": ["m1", "m2"],
  "frames": [[0.0, 0.1], [0.02, 0.12]]
}
```

**CSV** (a leading `frame`/`index`/`time` column is treated as an index and
dropped; `# units: deg` comment switches units):

```
frame,m1,m2
0,0.0,0.1
1,0.02,0.12
```

The parser validates the file (rectangular frames, finite numbers, ≥1 channel)
and raises a clear error shown in the metadata line if the file is malformed.

---

## 7. Commanded vs actual

* **Actual** angle comes from the motor's feedback (`statusUpdated.position`).
* **Commanded** angle is whatever the screen last told the motor to go to
  (manual Send, A/B step, or sequence frame). The screen knows it because every
  motion on this screen originates here.

The row shows both (`∠ commanded → actual`) and the position bar draws a ghost
marker for commanded and a solid marker for actual, so the two are never
confused. On **enable**, the commanded ghost snaps to the current actual angle,
matching the worker's safe-enable (the motor comes up holding where it is).

---

## 8. Safety backstops (all in the worker, unchanged)

These apply to commands from *either* screen:

* **Range clamp** — every position command is clamped to the calibrated
  `[min, max]` before it reaches the bus. Velocity/current modes additionally get
  a feedback-driven cutout that disables the motor if it leaves the range.
* **Disabled motors ignore commands** — `SetTarget` only updates a stored target;
  the control loop skips any motor that isn't enabled. So A/B or a sequence
  posting to a disabled motor moves nothing.
* **Safe-enable** — enabling seeds the hold setpoint to the shaft's current
  position and re-asserts it after enabling, so a motor never jumps to a stale
  target on enable.
* **E-STOP** disables every motor and clears the dashboard's enabled state.

---

## 9. Totals bar

Recomputed on every status/power update:

* **Σ current** — sum of |current| across rows.
* **Σ torque** — sum of |torque| across rows.
* **active** — count of enabled motors.
* **cycling** — count of motors currently moving under A/B or sequence playback.

---

## 10. Running & testing

Run the app:

```bash
python3 main.py
```

Run the tests (Qt runs headless via the offscreen platform):

```bash
QT_QPA_PLATFORM=offscreen python -m pytest -q
```

Dashboard-specific tests:

```
tests/test_sequence.py          # JSON/CSV parsing + validation
tests/test_position_bar.py      # marker/direction geometry
tests/test_motor_row.py         # calibration, lock, A/B guard, commanded-vs-actual
tests/test_sequencer.py         # frame stepping, stop/abort
tests/test_dashboard_totals.py  # totals aggregation, play guards
```
