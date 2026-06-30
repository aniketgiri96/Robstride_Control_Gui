# SOP — RobStride Control GUI

Standard operating procedure for anyone new to this repo. Covers setup, running
the app, connecting motors, assigning CAN IDs, and troubleshooting the failures
you are most likely to hit.

---

## 1. What this is

A PySide6 desktop GUI to drive RobStride motors over a USB-CAN adapter.

Layers (low → high):

| File | Responsibility |
|------|----------------|
| `robstride_gui/protocol.py` | Frame encoding/decoding, AT framing, param IDs |
| `robstride_gui/transport.py` | Wire backends: `SerialATTransport` (USB-CAN), `SocketCANTransport` (can0) |
| `robstride_gui/bus.py` | High-level ops: ping/scan, enable, set-zero, set-id, inventory |
| `robstride_gui/worker.py` | Background thread; receives commands from the UI, polls motors |
| `robstride_gui/ui/` | Qt widgets: `main_window`, `motor_panel`, `device_dialog`, `plot_panel` |
| `main.py` | Entry point |

Mental model: **the UI never touches hardware directly.** It posts command
objects (`worker.py`) to a worker thread, which talks to the bus/transport.

---

## 2. First-time setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

The app needs **PySide6** and **pyserial**. If you see
`PySide6 is not installed`, your venv is not active — activate it or run with
`.venv/bin/python`.

Serial port permissions (Linux): the adapter is `/dev/ttyUSB0`, owned by group
`dialout`. Add yourself once, then log out/in:

```bash
sudo usermod -aG dialout $USER
```

---

## 3. Running the app

```bash
source .venv/bin/activate
python main.py
```

Only **one process** can hold `/dev/ttyUSB0` at a time (the transport takes an
exclusive lock). If a stale instance is running, the new one cannot open the
port. See §7 to free it.

---

## 4. Connecting to a motor

1. Power the motor from its **main DC supply** (24/48 V). USB powers only the
   adapter, never the motor.
2. In the app, pick the port (`/dev/ttyUSB0`) and click **Connect**.
3. Click **Detect** (Devices dialog). It scans CAN IDs and lists every
   responding motor with its unique 64-bit MCU id.
4. A tab appears per responding ID — select it to drive the motor.

> Detect auto-adds a tab for each responding CAN id. The **"Unique MCU id(s)"**
> column tells you how many *physical* motors are on one id: two MCU ids on one
> CAN id = a collision (see §6).

---

## 5. Assigning / changing a motor's CAN ID

RobStride firmware **adopts a new ID only after a power-cycle**. The app will say
*"new id not active yet. Power-cycle the motor, then Detect."* — that is normal,
not an error.

Procedure:

1. Set the new ID in the UI (Set Motor ID).
2. **Cut DC power to the motor fully**, wait ~3 s, power back on.
3. Click **Detect** — it should now answer at the new ID, persisted to flash.

---

## 6. Multiple motors (the #1 gotcha)

If two motors ship with the **same CAN id** (often 0), every command hits both —
they move in lock-step and you cannot split them on the bus. `SET_DEVICE_ID`
broadcasts to *all* motors sharing that id at once.

**Assign IDs one motor at a time, physically isolated:**

1. Power **only motor A** (motor B unpowered / unplugged from CAN).
2. Set A's new ID → power-cycle A → Detect (confirm A's new ID).
3. Power **only motor B**, set its ID → power-cycle B → Detect.
4. Power both — distinct IDs, separate tabs.

Detect's MCU-id column is how you confirm there is one motor per id.

---

## 7. Troubleshooting

### "PySide6 is not installed"
venv not active. Use `source .venv/bin/activate` or `.venv/bin/python main.py`.

### Cannot open port / port busy
A stale instance holds `/dev/ttyUSB0`. Find and kill it:

```bash
fuser -v /dev/ttyUSB0          # show holder PID
kill <PID>                     # then re-run; kill -9 only if it won't exit
```

### "No data coming" — motor never replies
Work top-down; the layers below are quick to rule out in software:

1. **Adapter present?** `lsusb | grep -i serial` — expect a CH340/CP210x/FTDI.
   `ls -l /dev/ttyUSB*` should show the device.
2. **Serial layer OK?** Port opens and frames go out but **zero bytes return at
   every baud** → the problem is *downstream of the USB adapter*, i.e. the CAN
   bus, not the app. (Default serial baud 921600; CAN bitrate 1 Mbps.)
3. **Then it is physical**, in order of likelihood:
   - Motor not powered (check its power LED).
   - CAN_H/CAN_L loose or swapped.
   - Missing 120 Ω bus termination.

The app/adapter/serial config can all be green while the motor is silent — that
isolates the fault to power/wiring.

### Two motors move together / only one tab
Same CAN id collision. Follow §6.

### Motor still spinning after closing the app
Releasing the serial port does **not** brake the motor — it only stops commands.
Power-cycle the motor to physically stop it.

---

## 8. Testing

Hardware-free; a `FakeTransport` stub replays frames so tests run with no motor.

```bash
PYTHONPATH=. .venv/bin/python -m pytest -q
```

Add tests under `tests/test_*.py`. Worker-level behavior can be tested by calling
`MotorWorker._apply(<Command>)` directly with a fake transport — no Qt event loop
needed.

---

## 9. Safe-operation checklist

- [ ] Motor mechanically secured before enabling.
- [ ] Start in a low-authority mode (Velocity/Position) with small setpoints.
- [ ] Know where the E-stop / Disable control is before sending motion.
- [ ] Only one motor on the bus when assigning IDs.
- [ ] Power-cycle to confirm an ID change stuck.
