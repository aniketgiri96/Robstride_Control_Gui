# RobStride Control GUI

A custom, robust desktop GUI for controlling RobStride motors (RS-00 ... RS-06,
default **RS-04**) — a replacement for the official MotorStudio AppImage.

It speaks the RobStride CAN 2.0 protocol over **two interchangeable transports**:

| Transport | Device | Notes |
|-----------|--------|-------|
| **Serial (AT)** | `/dev/ttyUSB*` @ 921600 | The path proven by the vendor `motor_zero.py` on this hardware. No kernel setup. |
| **SocketCAN** | `can0` @ 1 Mbit/s | Standard `python-can`; needs the interface brought up first. |

## Features

- Connect / disconnect, motor **scan**, enable / disable, **set mechanical zero**
- Control modes: **MIT** (impedance: pos + vel + Kp/Kd), **Position**, **Velocity**, **Current**
- **Live plots** of position / velocity / torque / temperature (pyqtgraph, ~30 FPS)
- **Multi-motor**: one tab per CAN id, controlled independently
- **Safety**: soft position bounds, velocity / current / torque caps, global **E-STOP**
- **Presets**: save / apply / delete named setpoints (`~/.config/robstride_gui/presets.json`)
- All serial / CAN IO runs on a worker thread, so the UI never freezes

## Install

```bash
cd Control_Gui
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
# give the adapter permission first (daily):
ls /dev/ttyUSB*            # find your port, e.g. /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0

source .venv/bin/activate
python main.py
```

Then in the app:

1. Pick **Transport** = *Serial (AT)* and select your `/dev/ttyUSB*` port
   (or *SocketCAN* + `can0`).
2. Click **Connect**, then **Scan** to discover motor ids (or **Add** an id manually).
3. Per motor tab: choose a **Mode**, move the target, click **Enable**.
4. **Set Zero** defines the current shaft angle as 0.
5. **E-STOP** immediately disables all motors.

### SocketCAN setup (only for the `can0` path)

```bash
sudo modprobe can
sudo ip link set can0 up type can bitrate 1000000
```

## Architecture

```
main.py
|- robstride_gui/
   |- protocol.py   # pure: comm types, register map, RS-04 scaling, frame encode/decode
   |- transport.py  # Transport ABC + SerialATTransport + SocketCANTransport
   |- bus.py        # ping/enable/disable/zero/read/write/MIT + pos/vel/current
   |- safety.py     # soft limits, caps, E-stop latch (clamps every command)
   |- presets.py    # JSON-backed motion presets
   |- worker.py     # QThread control loop; UI talks to it via Command objects
   |- ui/           # plot_panel, motor_panel, main_window
```

The protocol layer is transport-agnostic: every command is a
`(comm_type, extra_data, device_id, data)` tuple, serialized by the active
transport. The **AT frame format** was reverse-engineered and unit-tested
against the vendor `motor_zero.py` frames:

```
AT_frame = b"AT" + uint32_be((ext_id << 3) | 0x04) + dlc + data + b"\r\n"
ext_id   = (comm_type << 24) | (extra_data << 8) | device_id
```

## Test

```bash
source .venv/bin/activate
pytest -q          # 18 tests: framing, MIT scaling, status/param parse, presets
```

## Hardware verification note

The protocol math (extended-id packing, MIT scaling for RS-04, feedback decode)
is verified by unit tests and matches both the vendor SocketCAN SDK and the
captured AT frames. **Before trusting motion on real hardware**, do a first
power-on with the motor free to spin, low Kp/Kd, and a small target:

- Confirm **Scan**/ping returns your motor id.
- Confirm **Set Zero** + the position readout track the shaft by hand.
- Confirm a small **Position** step moves the expected direction/magnitude.

If position/velocity signs or scale look off for your specific firmware, the
only thing to adjust is the model scaling in `protocol.py` (the `MODEL_*`
tables) — the rest of the stack is model-driven.
