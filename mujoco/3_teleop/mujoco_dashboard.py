import mujoco, mujoco.viewer
import numpy as np
import tkinter as tk
from collections import deque
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import csv
import os, sys, socket, json, time   # --- motor bridge: added ---

# --- motor bridge: config ---------------------------------------------------
# The RobStride GUI must be running with its UdpTargetLink started on this
# host/port. This dashboard sends commanded joint angles (radians) there and
# reads the motor's measured state back. Nothing else in the GUI changes.
GUI_HOST, GUI_PORT = "127.0.0.1", 8642
MOTOR_HZ = 100.0                 # rate we push setpoints to the motor
_mot_period = 1.0 / MOTOR_HZ
_mot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_mot_sock.setblocking(False)     # never stall the sim loop on the network
_last_mot_send = 0.0
last_meas = {}                   # joint_name -> {"pos":rad, "vel":..., "torque":...}


def motor_send(targets_rad):
    """Fire one command datagram (fire-and-forget)."""
    try:
        _mot_sock.sendto(json.dumps({"targets": targets_rad}).encode("utf-8"),
                         (GUI_HOST, GUI_PORT))
    except OSError:
        pass


def motor_drain():
    """Non-blocking: return the newest motor-state dict waiting in the socket,
    or None. Draining rather than blocking keeps the sim smooth even if the GUI
    is slow or not answering."""
    latest = None
    while True:
        try:
            data, _ = _mot_sock.recvfrom(65535)
        except (BlockingIOError, OSError):
            break
        try:
            st = json.loads(data.decode("utf-8")).get("state")
            if isinstance(st, dict):
                latest = st
        except ValueError:
            pass
    return latest
# ---------------------------------------------------------------------------

# Pass a model path as the first argument; with no argument, default to the
# scene built in the sibling stage (mujoco/1_build_scene/scene.xml) so the
# dashboard "just runs" from the repo root.
_DEFAULT_SCENE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "1_build_scene", "scene.xml")
MODEL = sys.argv[1] if len(sys.argv) > 1 else _DEFAULT_SCENE   # --- added: allow a path ---
m = mujoco.MjModel.from_xml_path(MODEL)
d = mujoco.MjData(m)
nu = m.nu


class Trap:
    def __init__(self, q0, qf, vmax, amax):
        self.q0, self.qf = q0, qf
        D = qf - q0; self.sign = 1.0 if D >= 0 else -1.0
        D = abs(D); self.a = max(amax, 1e-6); vmax = max(vmax, 1e-6)
        t_acc = vmax/self.a; d_acc = 0.5*self.a*t_acc**2
        if 2*d_acc <= D:
            self.t_acc, self.vpk, self.d_acc = t_acc, vmax, d_acc
            self.t_cru = (D-2*d_acc)/vmax
        else:
            self.vpk = np.sqrt(self.a*D); self.t_acc = self.vpk/self.a
            self.d_acc = 0.5*self.a*self.t_acc**2; self.t_cru = 0.0
        self.T = 2*self.t_acc + self.t_cru
    def pos(self, t):
        if t <= 0: return self.q0
        if t >= self.T: return self.qf
        ta, tc, a = self.t_acc, self.t_cru, self.a
        if t < ta: dd = 0.5*a*t*t
        elif t < ta+tc: dd = self.d_acc + self.vpk*(t-ta)
        else:
            td = t-ta-tc; dd = self.d_acc + self.vpk*tc + self.vpk*td - 0.5*a*td*td
        return self.q0 + self.sign*dd


info = []
for a in range(nu):
    jid = m.actuator_trnid[a, 0]
    info.append({"a": a, "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a),
                 "q": m.jnt_qposadr[jid], "v": m.jnt_dofadr[jid],
                 # --- motor bridge: the JOINT name is the label the GUI keys on ---
                 "joint": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)})

profiles = [None]*nu
prof_t0 = [0.0]*nu
COLORS = ["C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7"]

seq = []; seq_idx = 0; seq_phase = "start"; phase_t0 = 0.0

BUF = 600
tbuf = deque(maxlen=BUF)
posb = [deque(maxlen=BUF) for _ in range(nu)]
velb = [deque(maxlen=BUF) for _ in range(nu)]
torb = [deque(maxlen=BUF) for _ in range(nu)]
measb = [deque(maxlen=BUF) for _ in range(nu)]   # --- motor bridge: measured pos (deg) ---

logfile = open("joint_log.csv", "w", newline="")
logw = csv.writer(logfile)
_hdr = ["time", "seq_step"]
for c in info:
    _hdr += [f"cmd_{c['name']}_deg", f"pos_{c['name']}_deg",
             f"vel_{c['name']}_degps", f"torque_{c['name']}_Nm"]
logw.writerow(_hdr)
LOG_EVERY = 10

root = tk.Tk(); root.title("6-Motor Control + Sequence + Live Data")
left = tk.Frame(root); left.pack(side="left", fill="y", padx=4, pady=4)

gp = tk.Frame(left); gp.pack(side="top", anchor="w")
tk.Label(gp, text="Max speed (deg/s):").pack(side="left")
e_vmax = tk.Entry(gp, width=6); e_vmax.insert(0, "30"); e_vmax.pack(side="left")
tk.Label(gp, text=" Accel:").pack(side="left")
e_acc = tk.Entry(gp, width=6); e_acc.insert(0, "60"); e_acc.pack(side="left")
def params():
    try: return float(e_vmax.get()), float(e_acc.get())
    except ValueError: return 30.0, 60.0

sliders = []; entries = []
def start(i):
    try: qf = float(entries[i].get())
    except ValueError: return
    lo, hi = m.actuator_ctrlrange[i]; qf = float(np.clip(qf, lo, hi))
    vmax, amax = params()
    profiles[i] = Trap(sliders[i].get(), qf, vmax, amax); prof_t0[i] = d.time
def move_all():
    for i in range(nu): start(i)

for i in range(nu):
    lo, hi = m.actuator_ctrlrange[i]
    row = tk.Frame(left); row.pack(side="top", anchor="w")
    tk.Label(row, text=info[i]["name"], width=12, anchor="w").pack(side="left")
    s = tk.Scale(row, from_=round(float(lo),1), to=round(float(hi),1),
                 orient="horizontal", resolution=0.1, length=220)
    s.set(round(float(d.ctrl[i]),1)); s.pack(side="left"); sliders.append(s)
    tk.Label(row, text="tgt:").pack(side="left")
    e = tk.Entry(row, width=6); e.insert(0, "0"); e.pack(side="left"); entries.append(e)
    tk.Button(row, text="Move", command=lambda i=i: start(i)).pack(side="left")
tk.Button(left, text="Move ALL", command=move_all).pack(side="top", anchor="w", pady=2)

sf = tk.LabelFrame(left, text="Repeat sequence"); sf.pack(side="top", fill="x", pady=6)
listbox = tk.Listbox(sf, height=6, width=34); listbox.pack(side="top")
def capture_step():
    pose = np.array([sliders[i].get() for i in range(nu)])
    seq.append(pose); listbox.insert("end", " ".join(f"{v:.0f}" for v in pose))
def delete_step():
    sel = listbox.curselection()
    if sel: listbox.delete(sel[0]); del seq[sel[0]]
def clear_seq():
    seq.clear(); listbox.delete(0, "end")
bf = tk.Frame(sf); bf.pack(side="top", anchor="w")
tk.Button(bf, text="Capture step", command=capture_step).pack(side="left")
tk.Button(bf, text="Delete", command=delete_step).pack(side="left")
tk.Button(bf, text="Clear", command=clear_seq).pack(side="left")
df = tk.Frame(sf); df.pack(side="top", anchor="w")
tk.Label(df, text="Hold at each pose (s):").pack(side="left")
e_dwell = tk.Entry(df, width=5); e_dwell.insert(0, "0.5"); e_dwell.pack(side="left")
def dwell_time():
    try: return float(e_dwell.get())
    except ValueError: return 0.5
play_var = tk.BooleanVar(value=False)
def on_play():
    global seq_idx, seq_phase
    if play_var.get(): seq_idx = 0; seq_phase = "start"
tk.Checkbutton(sf, text="Play sequence (loop)", variable=play_var, command=on_play).pack(side="top", anchor="w")
seq_status = tk.Label(sf, text="stopped"); seq_status.pack(side="top", anchor="w")

log_var = tk.BooleanVar(value=True)
tk.Checkbutton(left, text="Log to joint_log.csv", variable=log_var).pack(side="top", anchor="w")

# --- motor bridge: on/off gate + live readout ------------------------------
mot_var = tk.BooleanVar(value=False)   # OFF by default so the motor never jumps on launch
tk.Checkbutton(left, text="Stream to real motor (UDP)", variable=mot_var).pack(side="top", anchor="w")
mot_status = tk.Label(left, text="motor: (off)", anchor="w", justify="left")
mot_status.pack(side="top", anchor="w")
# ---------------------------------------------------------------------------

pf = tk.Frame(root); pf.pack(side="right", fill="both", expand=True)
fig = Figure(figsize=(7, 6), dpi=90)
ax_p = fig.add_subplot(311); ax_v = fig.add_subplot(312); ax_t = fig.add_subplot(313)
ax_p.set_ylabel("pos (deg)"); ax_v.set_ylabel("vel (deg/s)"); ax_t.set_ylabel("torque (N*m)")
ax_t.set_xlabel("time (s)")
lp = []; lv = []; lt = []; lr = []   # lr = real/measured overlay
for i in range(nu):
    c = COLORS[i % len(COLORS)]
    lp.append(ax_p.plot([], [], color=c, label=info[i]["name"])[0])
    lv.append(ax_v.plot([], [], color=c)[0])
    lt.append(ax_t.plot([], [], color=c)[0])
    # --- motor bridge: dashed line = real motor measured position ---
    lr.append(ax_p.plot([], [], color=c, linestyle="--", linewidth=0.8)[0])
ax_p.legend(fontsize=6, ncol=3, loc="upper left")
fig.tight_layout()
canvas = FigureCanvasTkAgg(fig, master=pf)
canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

step = 0
PLOT_EVERY = 25
with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        if play_var.get() and seq:
            if seq_phase == "start":
                vmax, amax = params()
                for i in range(nu):
                    profiles[i] = Trap(sliders[i].get(), float(seq[seq_idx][i]), vmax, amax)
                    prof_t0[i] = d.time
                seq_phase = "moving"
                seq_status.config(text=f"playing step {seq_idx+1}/{len(seq)}")
            elif seq_phase == "moving":
                if all(p is None for p in profiles):
                    seq_phase = "dwelling"; phase_t0 = d.time
            elif seq_phase == "dwelling":
                if d.time - phase_t0 >= dwell_time():
                    seq_idx = (seq_idx + 1) % len(seq); seq_phase = "start"
        elif not play_var.get():
            seq_status.config(text="stopped")

        for i in range(nu):
            if profiles[i] is not None:
                tt = d.time - prof_t0[i]
                sliders[i].set(round(float(profiles[i].pos(tt)), 1))
                if tt >= profiles[i].T: profiles[i] = None
            d.ctrl[i] = sliders[i].get()

        # --- motor bridge: push commanded angles to the real motor, read back ---
        now = time.monotonic()
        if mot_var.get() and (now - _last_mot_send) >= _mot_period:
            _last_mot_send = now
            # d.ctrl is radians (MuJoCo internal), which is what the motor wants.
            # Send every joint by name; the GUI drives only the ones in its map.
            targets = {info[i]["joint"]: float(d.ctrl[i]) for i in range(nu)}
            motor_send(targets)
            st = motor_drain()
            if st:
                last_meas.update(st)
            shown = ", ".join(
                f"{n}={v.get('pos', float('nan')):+.3f}rad" for n, v in last_meas.items())
            mot_status.config(text=f"motor: {shown or 'no reply'}")
        elif not mot_var.get():
            mot_status.config(text="motor: (off)")

        mujoco.mj_step(m, d); viewer.sync()

        tbuf.append(d.time)
        for i in range(nu):
            posb[i].append(np.degrees(d.qpos[info[i]["q"]]))
            velb[i].append(np.degrees(d.qvel[info[i]["v"]]))
            torb[i].append(d.qfrc_actuator[info[i]["v"]])
            # --- motor bridge: last known measured pos (deg) for this joint, or NaN ---
            mv = last_meas.get(info[i]["joint"])
            measb[i].append(np.degrees(mv["pos"]) if mv else np.nan)
        if step % PLOT_EVERY == 0 and len(tbuf) > 1:
            ts = np.fromiter(tbuf, float)
            for i in range(nu):
                lp[i].set_data(ts, np.fromiter(posb[i], float))
                lv[i].set_data(ts, np.fromiter(velb[i], float))
                lt[i].set_data(ts, np.fromiter(torb[i], float))
                lr[i].set_data(ts, np.fromiter(measb[i], float))   # real overlay
            for ax in (ax_p, ax_v, ax_t):
                ax.relim(); ax.autoscale_view()
            canvas.draw_idle()
        step += 1
        if log_var.get() and step % LOG_EVERY == 0:
                lbl = f"step{seq_idx+1}" if (play_var.get() and seq) else "manual"
                row = [round(d.time, 4), lbl]
                for c in info:
                    row += [round(d.ctrl[c["a"]], 3), round(np.degrees(d.qpos[c["q"]]), 3),
                            round(np.degrees(d.qvel[c["v"]]), 3), round(d.qfrc_actuator[c["v"]], 3)]
                logw.writerow(row); logfile.flush()
        try: root.update()
        except tk.TclError: break
logfile.close()
print("saved joint_log.csv")