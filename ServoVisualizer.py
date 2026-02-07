import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time
import math

BAUD = 115200

# --- Manual slider throttle (keeps manual UI from spamming serial) ---
SEND_INTERVAL_SEC = 0.03

ALL_FINGERS_NAME = "All Fingers"
WAVE_ORDER = ["Pinky", "Ring", "Middle", "Pointer"]  # required order

# Wrist servos (270 degrees)
WRIST1_CH = 11
WRIST2_CH = 12
WRIST1_INIT = 135.0
WRIST2_INIT = 135.0
WRIST_RANGE = 270.0

# Finger order MUST match Arduino:
# 0=Pinky, 1=Ring, 2=Middle, 3=Pointer, 4=Thumb
FINGERS = [
    {"name": "Pinky",   "bottom_ch": 8, "bottom_init": 140.0,  "top_ch": 4, "top_init": 130.0, "extra_ch": None, "extra_init": None},
    {"name": "Ring",    "bottom_ch": 7, "bottom_init": 55.0, "top_ch": 3, "top_init": 140.0,  "extra_ch": None, "extra_init": None},
    {"name": "Middle",  "bottom_ch": 2, "bottom_init": 30.0, "top_ch": 1, "top_init": 140.0, "extra_ch": None, "extra_init": None},
    {"name": "Pointer", "bottom_ch": 5, "bottom_init": 145.0,  "top_ch": 0, "top_init": 40.0, "extra_ch": None, "extra_init": None},
    {"name": "Thumb",   "bottom_ch": 9, "bottom_init": 60.0, "top_ch": 10, "top_init": 60.0, "extra_ch": 6,   "extra_init": 80.0},
]

# --- Animation definitions (uncurled->curled angles) ---
ANIM_TABLE = {
    "Pinky":   {"bottom_uncurled": 90.0,  "bottom_curled": 90.0, "top_uncurled": 90.0, "top_curled": 90.0},
    "Ring":    {"bottom_uncurled": 90.0, "bottom_curled": 90.0,   "top_uncurled": 90.0,  "top_curled": 90.0},
    "Middle":  {"bottom_uncurled": 90.0, "bottom_curled": 90.0,  "top_uncurled": 90.0, "top_curled": 90.0},
    "Pointer": {"bottom_uncurled": 90.0,  "bottom_curled": 90.0, "top_uncurled": 90.0, "top_curled": 90.0},
}

THUMB_TOUCH_POSES = {
    "Pointer": {
        "target": {"bottom": 110.0, "top": 115.0},
        "thumb":  {"bottom": 140.0, "top": 115.0, "extra": 130.0},
    },
    "Middle": {
        "target": {"bottom": 115.0, "top": 105.0},
        "thumb":  {"bottom": 125.0, "top": 110.0, "extra": 138.0},
    },
    "Ring": {
        "target": {"bottom": 35.0, "top": 80.0},
        "thumb":  {"bottom": 130.0, "top": 105.0, "extra": 153.0},
    },
    "Pinky": {
        "target": {"bottom": 100.0, "top": 110.0},
        "thumb":  {"bottom": 125.0, "top": 110.0, "extra": 167.0},
    },
}

DEFAULT_FINGER_IDX = 3  # Pointer


def ease_in_out(t: float) -> float:
    """Cosine ease-in-out: smooth wave-like motion, t in [0,1]."""
    t = max(0.0, min(1.0, t))
    return 0.5 - 0.5 * math.cos(math.pi * t)


class ScrollableFrame(ttk.Frame):
    """
    A ttk.Frame that becomes vertically scrollable.
    Put your content inside: self.inner
    """
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)

        self.canvas = tk.Canvas(self, highlightthickness=0)
        self.vsb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vsb.set)

        self.vsb.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.inner = ttk.Frame(self.canvas)
        self.window_id = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")

        self.inner.bind("<Configure>", self._on_inner_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

        # Mouse wheel scrolling (global bind, simple + works)
        self._bind_mousewheel(self.canvas)

    def _on_inner_configure(self, _event=None):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self.canvas.itemconfigure(self.window_id, width=event.width)

    def _bind_mousewheel(self, widget):
        widget.bind_all("<MouseWheel>", self._on_mousewheel)      # Windows/macOS
        widget.bind_all("<Button-4>", self._on_mousewheel_linux)  # Linux up
        widget.bind_all("<Button-5>", self._on_mousewheel_linux)  # Linux down

    def _on_mousewheel(self, event):
        self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def _on_mousewheel_linux(self, event):
        if event.num == 4:
            self.canvas.yview_scroll(-1, "units")
        elif event.num == 5:
            self.canvas.yview_scroll(1, "units")


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Robot Hand Control (Manual + Animations) + Pot Reader")
        self.root.geometry("720x680")

        self.ser = None
        self.updating_sliders = False

        # last send times per logical channel (manual throttle)
        self.last_send_time = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}  # 0=top,1=bottom,2=thumb extra,3=wrist1,4=wrist2

        # Remember angles per finger (manual)
        self.top_angles = [f["top_init"] for f in FINGERS]
        self.bottom_angles = [f["bottom_init"] for f in FINGERS]
        self.extra_angles = [f["extra_init"] if f["extra_init"] is not None else None for f in FINGERS]

        # Wrist angles (global, not per-finger)
        self.wrist1_angle = WRIST1_INIT
        self.wrist2_angle = WRIST2_INIT

        self.finger_idx = tk.IntVar(value=DEFAULT_FINGER_IDX)

        # ---------- Animation runtime state ----------
        self.anim_running = False
        self.anim_after_ids = []
        self.anim_mode = None  # "single" | "all_fingers" | "curl_all"

        self.anim_selected_name = tk.StringVar(value="Pinky")

        self.anim_duration_ms = tk.IntVar(value=650)   # motion duration per segment
        self.anim_delay1_ms = tk.IntVar(value=300)     # bottom curl start -> top curl start
        self.anim_delay2_ms = tk.IntVar(value=450)     # top curl start -> bottom uncurl start
        self.anim_delay3_ms = tk.IntVar(value=250)     # bottom uncurl start -> top uncurl start
        self.anim_frame_ms = tk.IntVar(value=20)       # update period

        self.anim_loop = tk.BooleanVar(value=False)
        self.anim_loop_gap_ms = tk.IntVar(value=350)

        # NEW: delay between starting wave on each finger in All Fingers mode
        self.anim_between_fingers_ms = tk.IntVar(value=450)

        # For reducing serial spam in multi-finger mode
        self._last_sent = {}  # key=(finger_idx, logical_channel) -> last_angle

        # ---------- Connection bar ----------
        top = ttk.Frame(root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Serial Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.pack(side="left", padx=8)

        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=8)

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side="left", padx=10)

        # ---------- Pot display (always visible) ----------
        pot_frame = ttk.LabelFrame(root, text="Potentiometers (read-only)", padding=10)
        pot_frame.pack(fill="x", padx=10, pady=(0, 10))

        self.pot0_var = tk.StringVar(value="A0 (top): raw=----  V=---.-")
        self.pot1_var = tk.StringVar(value="A1 (bottom): raw=----  V=---.-")

        ttk.Label(pot_frame, textvariable=self.pot0_var, font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(0, 6))
        ttk.Label(pot_frame, textvariable=self.pot1_var, font=("Segoe UI", 10, "bold")).pack(anchor="w")

        # ---------- Notebook tabs ----------
        self.nb = ttk.Notebook(root)
        self.nb.pack(fill="both", expand=True, padx=10, pady=10)

        self.manual_tab = ttk.Frame(self.nb)
        self.anim_tab = ttk.Frame(self.nb)
        self.nb.add(self.manual_tab, text="Manual Control")
        self.nb.add(self.anim_tab, text="Animations")

        self._build_manual_tab(self.manual_tab)
        self._build_anim_tab(self.anim_tab)

        self.refresh_ports()
        self.apply_finger_ui(DEFAULT_FINGER_IDX, push_to_arduino=False)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- UI: Manual tab ----------------
    def _build_manual_tab(self, parent):
        finger_frame = ttk.LabelFrame(parent, text="Finger Selection", padding=10)
        finger_frame.pack(fill="x", padx=10, pady=(10, 10))

        ttk.Label(finger_frame, text="Finger:").pack(side="left")

        self.finger_values = [f["name"] for f in FINGERS] + ["Animations"]
        self.finger_combo = ttk.Combobox(
            finger_frame, state="readonly", width=18, values=self.finger_values
        )
        self.finger_combo.pack(side="left", padx=8)
        self.finger_combo.current(DEFAULT_FINGER_IDX)
        self.finger_combo.bind("<<ComboboxSelected>>", self.on_finger_selected)

        self.map_var = tk.StringVar(value="")
        ttk.Label(finger_frame, textvariable=self.map_var).pack(side="left", padx=12)

        self.top_group = ttk.LabelFrame(parent, text="Top Joint — 0–180°", padding=10)
        self.top_group.pack(fill="x", padx=10, pady=(0, 8))
        self.top_lbl = tk.StringVar(value="Angle: ----.-°")
        ttk.Label(self.top_group, textvariable=self.top_lbl).pack(anchor="w")
        self.top_slider = ttk.Scale(self.top_group, from_=0, to=180, orient="horizontal", command=self.on_top_slider)
        self.top_slider.pack(fill="x", pady=6)

        self.bottom_group = ttk.LabelFrame(parent, text="Bottom Joint — 0–180°", padding=10)
        self.bottom_group.pack(fill="x", padx=10, pady=(0, 8))
        self.bottom_lbl = tk.StringVar(value="Angle: ----.-°")
        ttk.Label(self.bottom_group, textvariable=self.bottom_lbl).pack(anchor="w")
        self.bottom_slider = ttk.Scale(self.bottom_group, from_=0, to=180, orient="horizontal", command=self.on_bottom_slider)
        self.bottom_slider.pack(fill="x", pady=6)

        self.extra_group = ttk.LabelFrame(parent, text="Thumb Extra — 0–180°", padding=10)
        self.extra_lbl = tk.StringVar(value="Angle: ----.-°")
        ttk.Label(self.extra_group, textvariable=self.extra_lbl).pack(anchor="w")
        self.extra_slider = ttk.Scale(self.extra_group, from_=0, to=180, orient="horizontal", command=self.on_extra_slider)
        self.extra_slider.pack(fill="x", pady=6)

        # ---------- Wrist controls (global, not per-finger) ----------
        wrist_frame = ttk.LabelFrame(parent, text="Wrist Servos (270° range) — Global", padding=10)
        wrist_frame.pack(fill="x", padx=10, pady=(0, 8))

        # Wrist 1 (ch 11)
        wrist1_group = ttk.LabelFrame(wrist_frame, text=f"Wrist 1 (PCA ch {WRIST1_CH}) — 0–270°", padding=6)
        wrist1_group.pack(fill="x", pady=(0, 6))
        self.wrist1_lbl = tk.StringVar(value=f"Angle: {WRIST1_INIT:.1f}°")
        ttk.Label(wrist1_group, textvariable=self.wrist1_lbl).pack(anchor="w")
        self.wrist1_slider = ttk.Scale(wrist1_group, from_=0, to=WRIST_RANGE, orient="horizontal", command=self.on_wrist1_slider)
        self.wrist1_slider.pack(fill="x", pady=4)
        self.wrist1_slider.set(WRIST1_INIT)

        # Wrist 2 (ch 12)
        wrist2_group = ttk.LabelFrame(wrist_frame, text=f"Wrist 2 (PCA ch {WRIST2_CH}) — 0–270°", padding=6)
        wrist2_group.pack(fill="x", pady=(0, 6))
        self.wrist2_lbl = tk.StringVar(value=f"Angle: {WRIST2_INIT:.1f}°")
        ttk.Label(wrist2_group, textvariable=self.wrist2_lbl).pack(anchor="w")
        self.wrist2_slider = ttk.Scale(wrist2_group, from_=0, to=WRIST_RANGE, orient="horizontal", command=self.on_wrist2_slider)
        self.wrist2_slider.pack(fill="x", pady=4)
        self.wrist2_slider.set(WRIST2_INIT)

        hint = ttk.Label(
            parent,
            text="Protocol:\n  F:<idx> selects finger\n  0:<deg> sets TOP, 1:<deg> sets BOTTOM, 2:<deg> sets THUMB EXTRA\n  3:<deg> sets WRIST1 (ch11, 0-270°), 4:<deg> sets WRIST2 (ch12, 0-270°)\nPots stream back as POT,a0_raw,a1_raw,a0_v,a1_v",
            foreground="#444"
        )
        hint.pack(anchor="w", padx=12, pady=12)

    # ---------------- UI: Animations tab ----------------
    def _build_anim_tab(self, parent):
        sf = ScrollableFrame(parent)
        sf.pack(fill="both", expand=True)

        wrap = ttk.Frame(sf.inner, padding=10)
        wrap.pack(fill="both", expand=True)

        # --- Selection ---
        sel = ttk.LabelFrame(wrap, text="Choose Animation Target", padding=10)
        sel.pack(fill="x", pady=(0, 10))

        ttk.Label(sel, text="Target:").pack(side="left")

        anim_values = list(ANIM_TABLE.keys()) + [ALL_FINGERS_NAME]
        self.anim_combo = ttk.Combobox(
            sel,
            state="readonly",
            width=18,
            values=anim_values,
            textvariable=self.anim_selected_name
        )
        self.anim_combo.pack(side="left", padx=8)
        self.anim_combo.set("Pinky")
        self.anim_combo.bind("<<ComboboxSelected>>", lambda e: self.on_anim_target_changed())

        self.anim_info_var = tk.StringVar(value="")
        ttk.Label(sel, textvariable=self.anim_info_var).pack(side="left", padx=12)

        # --- Hand actions ---
        actions = ttk.LabelFrame(wrap, text="Hand Actions", padding=10)
        actions.pack(fill="x", pady=(0, 10))

        ttk.Button(actions, text="Reset Hand to Init", command=self.reset_hand_to_init).pack(side="left")
        ttk.Button(actions, text="Curl 4 Fingers (Pinky/Ring/Middle/Pointer)", command=self.curl_four_fingers).pack(side="left", padx=8)

        # --- Timing controls ---
        timing = ttk.LabelFrame(wrap, text="Wave Timing Controls", padding=10)
        timing.pack(fill="x", pady=(0, 10))

        self._add_labeled_scale(timing, "Move duration per segment (ms)", self.anim_duration_ms, 100, 2000)
        self._add_labeled_scale(timing, "Delay 1: bottom curl start → top curl start (ms)", self.anim_delay1_ms, 0, 2000)
        self._add_labeled_scale(timing, "Delay 2: top curl start → bottom uncurl start (ms)", self.anim_delay2_ms, 0, 3000)
        self._add_labeled_scale(timing, "Delay 3: bottom uncurl start → top uncurl start (ms)", self.anim_delay3_ms, 0, 3000)
        self._add_labeled_scale(timing, "Frame update period (ms) (lower = smoother, more serial traffic)", self.anim_frame_ms, 10, 60)

        # --- All Fingers specific settings (show only when selected) ---
        self.allfingers_group = ttk.LabelFrame(wrap, text="All Fingers Wave Settings", padding=10)
        # (packed/unpacked dynamically)
        self._add_labeled_scale(
            self.allfingers_group,
            "Delay between starting each finger wave (ms) (controls how the wave travels)",
            self.anim_between_fingers_ms,
            0, 1500
        )

        # --- Thumb touch buttons (only shown when Target = All Fingers) ---
        thumb_touch = ttk.LabelFrame(self.allfingers_group, text="Thumb Touch (Presets)", padding=10)
        thumb_touch.pack(fill="x", pady=(8, 0))

        ttk.Button(thumb_touch, text="Thumb → Pointer", command=lambda: self.thumb_touch("Pointer")).pack(side="left")
        ttk.Button(thumb_touch, text="Thumb → Middle",  command=lambda: self.thumb_touch("Middle")).pack(side="left", padx=6)
        ttk.Button(thumb_touch, text="Thumb → Ring",    command=lambda: self.thumb_touch("Ring")).pack(side="left", padx=6)
        ttk.Button(thumb_touch, text="Thumb → Pinky",   command=lambda: self.thumb_touch("Pinky")).pack(side="left", padx=6)

        # --- Loop controls ---
        loopf = ttk.LabelFrame(wrap, text="Loop", padding=10)
        loopf.pack(fill="x", pady=(0, 10))

        ttk.Checkbutton(loopf, text="Loop animation", variable=self.anim_loop).pack(side="left")
        ttk.Label(loopf, text="Gap (ms):").pack(side="left", padx=(14, 6))
        ttk.Entry(loopf, textvariable=self.anim_loop_gap_ms, width=7).pack(side="left")

        # --- Buttons ---
        btns = ttk.Frame(wrap)
        btns.pack(fill="x", pady=(0, 10))

        self.anim_play_btn = ttk.Button(btns, text="Play", command=self.start_animation)
        self.anim_play_btn.pack(side="left")

        self.anim_stop_btn = ttk.Button(btns, text="Stop", command=self.stop_animation, state="disabled")
        self.anim_stop_btn.pack(side="left", padx=8)

        self.anim_status_var = tk.StringVar(value="Idle.")
        ttk.Label(wrap, textvariable=self.anim_status_var, foreground="#444").pack(anchor="w")

        self.on_anim_target_changed()

        expl = (
            "Single-finger wave timeline:\n"
            "  t0 = 0ms:          bottom curls (uncurled → curled)\n"
            "  t1 = Delay 1:      top curls    (uncurled → curled)\n"
            "  t2 = Delay 1+2:    bottom uncurls (curled → uncurled)\n"
            "  t3 = Delay 1+2+3:  top uncurls  (curled → uncurled)\n\n"
            "All Fingers mode:\n"
            "  Starts the above wave on Pinky, then Ring, then Middle, then Pointer.\n"
            "  The 'Delay between starting each finger wave' sets the offset between finger starts.\n"
            "  If that delay is short, waves overlap → smoother traveling wave."
        )
        ttk.Label(wrap, text=expl, foreground="#444").pack(anchor="w", pady=(8, 12))

    def _add_labeled_scale(self, parent, label, var, from_, to):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=4)
        ttk.Label(row, text=label).pack(anchor="w")
        s = ttk.Scale(row, from_=from_, to=to, orient="horizontal", command=lambda v: var.set(int(float(v))))
        s.pack(fill="x", pady=2)
        s.set(var.get())
        ttk.Label(row, textvariable=var, width=6).pack(anchor="e")

    def on_anim_target_changed(self):
        target = self.anim_selected_name.get()

        # show/hide All Fingers group
        if target == ALL_FINGERS_NAME:
            if not self.allfingers_group.winfo_ismapped():
                self.allfingers_group.pack(fill="x", pady=(0, 10))
            self.anim_info_var.set("(Wave travels Pinky → Ring → Middle → Pointer)")
        else:
            if self.allfingers_group.winfo_ismapped():
                self.allfingers_group.pack_forget()
            self._update_anim_info()

    def _update_anim_info(self):
        name = self.anim_selected_name.get()
        d = ANIM_TABLE.get(name, None)
        if not d:
            self.anim_info_var.set("")
            return
        self.anim_info_var.set(
            f"(Bottom {d['bottom_uncurled']}→{d['bottom_curled']} | Top {d['top_uncurled']}→{d['top_curled']})"
        )

    # ---------------- Serial: ports/connect ----------------
    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        items = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo["values"] = items
        if items and self.port_var.get() not in items:
            self.port_var.set(items[0])
        if not items:
            self.port_var.set("")

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        sel = self.port_var.get()
        if not sel:
            messagebox.showerror("No Port", "No serial port selected.")
            return
        port = sel.split(" - ")[0].strip()

        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.01)
            time.sleep(1.5)
            self.status_var.set(f"Connected: {port}")
            self.connect_btn.config(text="Disconnect")

            idx = self.finger_idx.get()
            self.send_finger(idx, force=True)
            self.send_angle(0, self.top_angles[idx], force=True)
            self.send_angle(1, self.bottom_angles[idx], force=True)
            if self.has_extra(idx):
                self.send_angle(2, float(self.extra_angles[idx]), force=True)

            # Send wrist init positions
            self.send_angle(3, self.wrist1_angle, force=True)
            self.send_angle(4, self.wrist2_angle, force=True)

            self.root.after(20, self.poll_serial)

        except Exception as e:
            self.ser = None
            messagebox.showerror("Connect failed", str(e))
            self.status_var.set("Disconnected")
            self.connect_btn.config(text="Connect")

    def disconnect(self):
        self.stop_animation()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.status_var.set("Disconnected")
        self.connect_btn.config(text="Connect")

    # ---------------- Serial: send helpers ----------------
    def _write_serial(self, text: str):
        if not (self.ser and self.ser.is_open):
            return
        try:
            self.ser.write(text.encode("utf-8"))
        except Exception as e:
            messagebox.showerror("Serial write failed", str(e))
            self.disconnect()

    def send_finger(self, finger_index: int, force: bool = False):
        if not (self.ser and self.ser.is_open):
            return
        self._write_serial(f"F:{finger_index}\n")

    def send_angle(self, logical_channel: int, angle: float, force: bool = False):
        """
        logical_channel:
          0 = TOP joint of currently selected finger
          1 = BOTTOM joint of currently selected finger
          2 = EXTRA servo (Thumb only; Arduino ignores otherwise)

        force=True bypasses the manual throttle (needed for smooth animation).
        """
        if not (self.ser and self.ser.is_open):
            return

        now = time.time()
        if (not force) and (now - self.last_send_time[logical_channel] < SEND_INTERVAL_SEC):
            return

        self._write_serial(f"{logical_channel}:{angle:.1f}\n")
        self.last_send_time[logical_channel] = now

    def _send_finger_angle_fast(self, finger_index: int, logical_channel: int, angle: float, always=False):
        """
        For multi-finger updates: writes 'F:i' + '<logical>:<angle>' in one shot.
        Adds a small deadband so we don't spam nearly-identical angles.
        """
        key = (finger_index, logical_channel)
        prev = self._last_sent.get(key, None)
        if (not always) and (prev is not None) and abs(prev - angle) < 0.3:
            return
        self._last_sent[key] = angle
        self._write_serial(f"F:{finger_index}\n{logical_channel}:{angle:.1f}\n")

    # ---------------- Manual tab logic ----------------
    def has_extra(self, idx: int) -> bool:
        return FINGERS[idx]["extra_ch"] is not None

    def on_finger_selected(self, _event=None):
        choice = self.finger_combo.get()
        if choice == "Animations":
            self.nb.select(self.anim_tab)
            self.finger_combo.current(self.finger_idx.get())
            return

        new_idx = self.finger_combo.current()
        self.set_current_finger(new_idx, push_to_arduino=True)

    def set_current_finger(self, new_idx: int, push_to_arduino: bool):
        old_idx = self.finger_idx.get()

        if not self.updating_sliders:
            self.top_angles[old_idx] = float(self.top_slider.get())
            self.bottom_angles[old_idx] = float(self.bottom_slider.get())
            if self.has_extra(old_idx):
                self.extra_angles[old_idx] = float(self.extra_slider.get())

        self.finger_idx.set(new_idx)
        self.apply_finger_ui(new_idx, push_to_arduino=push_to_arduino)

    def apply_finger_ui(self, idx: int, push_to_arduino: bool):
        f = FINGERS[idx]
        extra_text = f" | Extra ch {f['extra_ch']}" if self.has_extra(idx) else ""
        self.map_var.set(f"(Bottom ch {f['bottom_ch']} | Top ch {f['top_ch']}{extra_text})")

        self.top_group.config(text=f"Top Joint (PCA ch {f['top_ch']}) — 0–180°")
        self.bottom_group.config(text=f"Bottom Joint (PCA ch {f['bottom_ch']}) — 0–180°")

        if self.has_extra(idx):
            self.extra_group.config(text=f"Thumb Extra (PCA ch {f['extra_ch']}) — 0–180°")
            if not self.extra_group.winfo_ismapped():
                self.extra_group.pack(fill="x", padx=10, pady=(0, 8))
        else:
            if self.extra_group.winfo_ismapped():
                self.extra_group.pack_forget()

        self.updating_sliders = True
        self.top_slider.set(self.top_angles[idx])
        self.bottom_slider.set(self.bottom_angles[idx])
        self.top_lbl.set(f"Angle: {self.top_angles[idx]:.1f}°")
        self.bottom_lbl.set(f"Angle: {self.bottom_angles[idx]:.1f}°")

        if self.has_extra(idx):
            if self.extra_angles[idx] is None:
                self.extra_angles[idx] = float(f["extra_init"])
            self.extra_slider.set(float(self.extra_angles[idx]))
            self.extra_lbl.set(f"Angle: {float(self.extra_angles[idx]):.1f}°")
        self.updating_sliders = False

        if push_to_arduino and (self.ser and self.ser.is_open):
            self.send_finger(idx, force=True)
            self.send_angle(0, self.top_angles[idx], force=True)
            self.send_angle(1, self.bottom_angles[idx], force=True)
            if self.has_extra(idx):
                self.send_angle(2, float(self.extra_angles[idx]), force=True)

    def on_top_slider(self, value):
        if self.updating_sliders or self.anim_running:
            return
        angle = float(value)
        idx = self.finger_idx.get()
        self.top_angles[idx] = angle
        self.top_lbl.set(f"Angle: {angle:.1f}°")
        self.send_angle(0, angle)

    def on_bottom_slider(self, value):
        if self.updating_sliders or self.anim_running:
            return
        angle = float(value)
        idx = self.finger_idx.get()
        self.bottom_angles[idx] = angle
        self.bottom_lbl.set(f"Angle: {angle:.1f}°")
        self.send_angle(1, angle)

    def on_extra_slider(self, value):
        if self.updating_sliders or self.anim_running:
            return
        idx = self.finger_idx.get()
        if not self.has_extra(idx):
            return
        angle = float(value)
        self.extra_angles[idx] = angle
        self.extra_lbl.set(f"Angle: {angle:.1f}°")
        self.send_angle(2, angle)

    def on_wrist1_slider(self, value):
        if self.updating_sliders or self.anim_running:
            return
        angle = float(value)
        self.wrist1_angle = angle
        self.wrist1_lbl.set(f"Angle: {angle:.1f}°")
        self.send_angle(3, angle)  # logical channel 3 = wrist1

    def on_wrist2_slider(self, value):
        if self.updating_sliders or self.anim_running:
            return
        angle = float(value)
        self.wrist2_angle = angle
        self.wrist2_lbl.set(f"Angle: {angle:.1f}°")
        self.send_angle(4, angle)  # logical channel 4 = wrist2

    def thumb_touch(self, target_name: str):
        """
        Moves the THUMB to touch the given target finger using THUMB_TOUCH_POSES constants.
        Smoothly animates: target finger (bottom+top) + thumb (bottom+top+extra).
        """
        if not (self.ser and self.ser.is_open):
            messagebox.showerror("Not connected", "Connect to the Arduino first.")
            return

        if target_name not in THUMB_TOUCH_POSES:
            messagebox.showerror("Missing preset", f"No thumb-touch preset found for '{target_name}'.")
            return

        if self.anim_running:
            return

        target_idx = self._finger_index_by_name(target_name)
        thumb_idx = self._finger_index_by_name("Thumb")
        if target_idx is None or thumb_idx is None:
            messagebox.showerror("Error", "Could not find target finger or Thumb in FINGERS list.")
            return

        # Stop anything else, then start this animation
        self.stop_animation()
        self.anim_running = True
        self.anim_mode = "thumb_touch"
        self.anim_after_ids = []
        self._last_sent = {}

        dur = int(self.anim_duration_ms.get())
        frame = int(self.anim_frame_ms.get())

        preset = THUMB_TOUCH_POSES[target_name]
        end_target_bottom = float(preset["target"]["bottom"])
        end_target_top = float(preset["target"]["top"])

        end_thumb_bottom = float(preset["thumb"]["bottom"])
        end_thumb_top = float(preset["thumb"]["top"])
        end_thumb_extra = float(preset["thumb"]["extra"])

        # Start values (use what the GUI currently thinks the angles are)
        start_target_bottom = float(self.bottom_angles[target_idx])
        start_target_top = float(self.top_angles[target_idx])

        start_thumb_bottom = float(self.bottom_angles[thumb_idx])
        start_thumb_top = float(self.top_angles[thumb_idx])
        # extra always exists on thumb in your setup; still guard for safety
        start_thumb_extra = float(self.extra_angles[thumb_idx]) if self.extra_angles[thumb_idx] is not None else float(FINGERS[thumb_idx]["extra_init"])

        # UI status + lock manual sliders
        self.anim_play_btn.config(state="disabled")
        self.anim_stop_btn.config(state="normal")
        self.anim_status_var.set(
            f"Thumb touching {target_name}…  (dur={dur}ms)"
        )
        self._set_manual_controls_enabled(False)

        # Animate target finger (bottom + top)
        self._schedule_move_for_finger(target_idx, 1, start_target_bottom, end_target_bottom, 0, dur, frame)
        self._schedule_move_for_finger(target_idx, 0, start_target_top,    end_target_top,    0, dur, frame)

        # Animate thumb (bottom + top + extra)
        self._schedule_move_for_finger(thumb_idx, 1, start_thumb_bottom, end_thumb_bottom, 0, dur, frame)
        self._schedule_move_for_finger(thumb_idx, 0, start_thumb_top,    end_thumb_top,    0, dur, frame)
        self._schedule_move_for_finger(thumb_idx, 2, start_thumb_extra,  end_thumb_extra,  0, dur, frame)

        # Update the stored angles immediately to match the new pose target
        self.bottom_angles[target_idx] = end_target_bottom
        self.top_angles[target_idx] = end_target_top

        self.bottom_angles[thumb_idx] = end_thumb_bottom
        self.top_angles[thumb_idx] = end_thumb_top
        self.extra_angles[thumb_idx] = end_thumb_extra

        # If user is currently viewing one of these fingers in Manual tab, refresh sliders
        cur = self.finger_idx.get()
        if cur in (target_idx, thumb_idx):
            self.apply_finger_ui(cur, push_to_arduino=False)

        self._after(dur + 40, self._animation_done)


    # ---------------- Hand Actions ----------------
    def reset_hand_to_init(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showerror("Not connected", "Connect to the Arduino first.")
            return

        self.stop_animation()

        # Update internal memory + push to Arduino
        for i, f in enumerate(FINGERS):
            self.top_angles[i] = float(f["top_init"])
            self.bottom_angles[i] = float(f["bottom_init"])
            if f["extra_ch"] is not None:
                self.extra_angles[i] = float(f["extra_init"])

            self.send_finger(i, force=True)
            self.send_angle(0, self.top_angles[i], force=True)
            self.send_angle(1, self.bottom_angles[i], force=True)
            if f["extra_ch"] is not None:
                self.send_angle(2, float(self.extra_angles[i]), force=True)

        # Refresh manual UI sliders to current selected finger
        self.apply_finger_ui(self.finger_idx.get(), push_to_arduino=False)
        self.anim_status_var.set("Reset hand to init positions.")

    def curl_four_fingers(self):
        """
        Smoothly curl Pinky/Ring/Middle/Pointer at the same time (both joints together).
        """
        if not (self.ser and self.ser.is_open):
            messagebox.showerror("Not connected", "Connect to the Arduino first.")
            return
        if self.anim_running:
            return

        self.stop_animation()
        self.anim_running = True
        self.anim_mode = "curl_all"
        self.anim_after_ids = []
        self._last_sent = {}

        dur = int(self.anim_duration_ms.get())
        frame = int(self.anim_frame_ms.get())

        self.anim_play_btn.config(state="disabled")
        self.anim_stop_btn.config(state="normal")
        self.anim_status_var.set("Curling Pinky/Ring/Middle/Pointer together…")
        self._set_manual_controls_enabled(False)

        # Schedule curl on each finger concurrently
        for fname in WAVE_ORDER:
            idx = self._finger_index_by_name(fname)
            a = ANIM_TABLE[fname]

            b0, b1 = a["bottom_uncurled"], a["bottom_curled"]
            t0, t1 = a["top_uncurled"], a["top_curled"]

            # Optionally snap to uncurled first (so motion is predictable)
            self._send_finger_angle_fast(idx, 1, b0, always=True)
            self._send_finger_angle_fast(idx, 0, t0, always=True)

            # Curl both joints together
            self._schedule_move_for_finger(idx, 1, b0, b1, 0, dur, frame)
            self._schedule_move_for_finger(idx, 0, t0, t1, 0, dur, frame)

        self._after(dur + 30, self._animation_done)

    # ---------------- Animation engine ----------------
    def start_animation(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showerror("Not connected", "Connect to the Arduino first.")
            return
        if self.anim_running:
            return

        target = self.anim_selected_name.get()

        if target == ALL_FINGERS_NAME:
            self.start_all_fingers_wave()
            return

        if target not in ANIM_TABLE:
            messagebox.showerror("No animation", "This target has no animation table entry.")
            return

        self.anim_mode = "single"
        self._start_single_finger_wave(target)

    def _start_single_finger_wave(self, finger_name: str):
        finger_index = self._finger_index_by_name(finger_name)
        if finger_index is None:
            messagebox.showerror("Error", f"Finger '{finger_name}' not found in FINGERS.")
            return

        a = ANIM_TABLE[finger_name]
        b0, b1 = a["bottom_uncurled"], a["bottom_curled"]
        t0, t1 = a["top_uncurled"], a["top_curled"]

        # Keep manual state consistent with animation start
        self.top_angles[finger_index] = t0
        self.bottom_angles[finger_index] = b0

        # Move UI to that finger (manual tab)
        self.finger_combo.current(finger_index)
        self.set_current_finger(finger_index, push_to_arduino=False)

        # Select finger and set start pose
        self.send_finger(finger_index, force=True)
        self.send_angle(1, b0, force=True)
        self.send_angle(0, t0, force=True)

        dur = int(self.anim_duration_ms.get())
        d1 = int(self.anim_delay1_ms.get())
        d2 = int(self.anim_delay2_ms.get())
        d3 = int(self.anim_delay3_ms.get())
        frame = int(self.anim_frame_ms.get())

        s_bottom_curl = 0
        s_top_curl = d1
        s_bottom_uncurl = d1 + d2
        s_top_uncurl = d1 + d2 + d3

        self.anim_after_ids = []
        self.anim_running = True
        self.anim_play_btn.config(state="disabled")
        self.anim_stop_btn.config(state="normal")
        self.anim_status_var.set(
            f"Running {finger_name} wave… (dur={dur}ms, d1={d1}ms, d2={d2}ms, d3={d3}ms)"
        )

        self._set_manual_controls_enabled(False)

        # Single-finger scheduling uses send_angle (no finger switching needed)
        self._schedule_move(logical_channel=1, start_deg=b0, end_deg=b1, start_ms=s_bottom_curl, duration_ms=dur, frame_ms=frame)
        self._schedule_move(logical_channel=0, start_deg=t0, end_deg=t1, start_ms=s_top_curl, duration_ms=dur, frame_ms=frame)
        self._schedule_move(logical_channel=1, start_deg=b1, end_deg=b0, start_ms=s_bottom_uncurl, duration_ms=dur, frame_ms=frame)
        self._schedule_move(logical_channel=0, start_deg=t1, end_deg=t0, start_ms=s_top_uncurl, duration_ms=dur, frame_ms=frame)

        end_ms = max(
            s_bottom_curl + dur,
            s_top_curl + dur,
            s_bottom_uncurl + dur,
            s_top_uncurl + dur,
        )
        self._after(end_ms + 20, self._animation_done)

    def start_all_fingers_wave(self):
        """
        Traveling wave: Pinky -> Ring -> Middle -> Pointer, then loops if enabled.
        """
        if not (self.ser and self.ser.is_open):
            messagebox.showerror("Not connected", "Connect to the Arduino first.")
            return
        if self.anim_running:
            return

        self.anim_mode = "all_fingers"
        self.anim_running = True
        self.anim_after_ids = []
        self._last_sent = {}

        dur = int(self.anim_duration_ms.get())
        d1 = int(self.anim_delay1_ms.get())
        d2 = int(self.anim_delay2_ms.get())
        d3 = int(self.anim_delay3_ms.get())
        frame = int(self.anim_frame_ms.get())
        between = int(self.anim_between_fingers_ms.get())

        # per-finger internal start times (relative inside each finger)
        s_bottom_curl = 0
        s_top_curl = d1
        s_bottom_uncurl = d1 + d2
        s_top_uncurl = d1 + d2 + d3

        per_finger_total = max(
            s_bottom_curl + dur,
            s_top_curl + dur,
            s_bottom_uncurl + dur,
            s_top_uncurl + dur,
        )

        self.anim_play_btn.config(state="disabled")
        self.anim_stop_btn.config(state="normal")
        self.anim_status_var.set(
            f"All Fingers wave… (between={between}ms, dur={dur}ms, d1={d1}, d2={d2}, d3={d3})"
        )
        self._set_manual_controls_enabled(False)

        # Pre-set each finger to its "uncurled" start so the wave is consistent
        for fname in WAVE_ORDER:
            idx = self._finger_index_by_name(fname)
            a = ANIM_TABLE[fname]
            self._send_finger_angle_fast(idx, 1, a["bottom_uncurled"], always=True)
            self._send_finger_angle_fast(idx, 0, a["top_uncurled"], always=True)

        # Schedule waves with offsets
        for k, fname in enumerate(WAVE_ORDER):
            base = k * between
            idx = self._finger_index_by_name(fname)
            a = ANIM_TABLE[fname]
            b0, b1 = a["bottom_uncurled"], a["bottom_curled"]
            t0, t1 = a["top_uncurled"], a["top_curled"]

            self._schedule_move_for_finger(idx, 1, b0, b1, base + s_bottom_curl, dur, frame)
            self._schedule_move_for_finger(idx, 0, t0, t1, base + s_top_curl, dur, frame)
            self._schedule_move_for_finger(idx, 1, b1, b0, base + s_bottom_uncurl, dur, frame)
            self._schedule_move_for_finger(idx, 0, t1, t0, base + s_top_uncurl, dur, frame)

        end_ms = (len(WAVE_ORDER) - 1) * between + per_finger_total
        self._after(end_ms + 30, self._animation_done)

    def stop_animation(self):
        if not self.anim_running:
            return

        for aid in list(self.anim_after_ids):
            try:
                self.root.after_cancel(aid)
            except Exception:
                pass
        self.anim_after_ids = []

        self.anim_running = False
        self.anim_mode = None
        self.anim_play_btn.config(state="normal")
        self.anim_stop_btn.config(state="disabled")
        self.anim_status_var.set("Stopped.")
        self._set_manual_controls_enabled(True)

    def _animation_done(self):
        if not self.anim_running:
            return

        # These actions should not auto-loop
        if self.anim_mode in ("curl_all", "thumb_touch"):
            self.anim_running = False
            self.anim_mode = None
            self.anim_play_btn.config(state="normal")
            self.anim_stop_btn.config(state="disabled")
            self.anim_status_var.set("Pose reached.")
            self._set_manual_controls_enabled(True)
            return


        if self.anim_loop.get():
            gap = int(self.anim_loop_gap_ms.get())
            self.anim_status_var.set(f"Looping… next run in {gap}ms")

            self.anim_running = False
            self.anim_mode = None
            self.anim_play_btn.config(state="normal")
            self.anim_stop_btn.config(state="disabled")
            self._set_manual_controls_enabled(True)

            self._after(gap, self.start_animation)
        else:
            self.anim_running = False
            self.anim_mode = None
            self.anim_play_btn.config(state="normal")
            self.anim_stop_btn.config(state="disabled")
            self.anim_status_var.set("Done.")
            self._set_manual_controls_enabled(True)

    def _set_manual_controls_enabled(self, enabled: bool):
        state = "normal" if enabled else "disabled"
        try:
            self.top_slider.state(["!disabled"] if enabled else ["disabled"])
            self.bottom_slider.state(["!disabled"] if enabled else ["disabled"])
            self.extra_slider.state(["!disabled"] if enabled else ["disabled"])
            self.wrist1_slider.state(["!disabled"] if enabled else ["disabled"])
            self.wrist2_slider.state(["!disabled"] if enabled else ["disabled"])
        except Exception:
            try:
                self.top_slider.config(state=state)
                self.bottom_slider.config(state=state)
                self.extra_slider.config(state=state)
                self.wrist1_slider.config(state=state)
                self.wrist2_slider.config(state=state)
            except Exception:
                pass

    def _after(self, ms: int, func):
        aid = self.root.after(ms, func)
        self.anim_after_ids.append(aid)
        return aid

    def _finger_index_by_name(self, name: str):
        for i, f in enumerate(FINGERS):
            if f["name"] == name:
                return i
        return None

    # --- Single-finger scheduler (uses current selected finger on Arduino) ---
    def _schedule_move(self, logical_channel: int, start_deg: float, end_deg: float,
                       start_ms: int, duration_ms: int, frame_ms: int):
        if duration_ms <= 0:
            self._after(start_ms, lambda: self.send_angle(logical_channel, end_deg, force=True))
            return

        def begin():
            if not self.anim_running:
                return
            t_start = time.perf_counter()

            def step():
                if not self.anim_running:
                    return
                elapsed = (time.perf_counter() - t_start) * 1000.0
                u = elapsed / float(duration_ms)
                if u >= 1.0:
                    self.send_angle(logical_channel, end_deg, force=True)
                    return
                e = ease_in_out(u)
                ang = start_deg + (end_deg - start_deg) * e
                self.send_angle(logical_channel, ang, force=True)
                self._after(frame_ms, step)

            step()

        self._after(start_ms, begin)

    # --- Multi-finger scheduler (selects finger each frame) ---
    def _schedule_move_for_finger(self, finger_index: int, logical_channel: int,
                                 start_deg: float, end_deg: float,
                                 start_ms: int, duration_ms: int, frame_ms: int):
        if duration_ms <= 0:
            self._after(start_ms, lambda: self._send_finger_angle_fast(finger_index, logical_channel, end_deg, always=True))
            return

        def begin():
            if not self.anim_running:
                return
            t_start = time.perf_counter()

            def step():
                if not self.anim_running:
                    return
                elapsed = (time.perf_counter() - t_start) * 1000.0
                u = elapsed / float(duration_ms)
                if u >= 1.0:
                    self._send_finger_angle_fast(finger_index, logical_channel, end_deg, always=True)
                    return
                e = ease_in_out(u)
                ang = start_deg + (end_deg - start_deg) * e
                self._send_finger_angle_fast(finger_index, logical_channel, ang, always=False)
                self._after(frame_ms, step)

            step()

        self._after(start_ms, begin)

    # ---------------- Serial polling ----------------
    def poll_serial(self):
        if not (self.ser and self.ser.is_open):
            return
        try:
            while True:
                line = self.ser.readline()
                if not line:
                    break
                self.handle_line(line.decode("utf-8", errors="replace").strip())
        except Exception as e:
            messagebox.showerror("Serial read failed", str(e))
            self.disconnect()
            return

        self.root.after(20, self.poll_serial)

    def handle_line(self, line: str):
        if not line.startswith("POT,"):
            return
        parts = line.split(",")
        if len(parts) != 5:
            return
        _, a0_raw, a1_raw, a0_v, a1_v = parts
        self.pot0_var.set(f"A0 (top): raw={a0_raw}  V={a0_v}")
        self.pot1_var.set(f"A1 (bottom): raw={a1_raw}  V={a1_v}")

    def on_close(self):
        try:
            self.stop_animation()
            if self.ser and self.ser.is_open:
                idx = self.finger_idx.get()
                self.send_finger(idx, force=True)
                self.send_angle(0, FINGERS[idx]["top_init"], force=True)
                self.send_angle(1, FINGERS[idx]["bottom_init"], force=True)
                if self.has_extra(idx):
                    self.send_angle(2, float(FINGERS[idx]["extra_init"]), force=True)

                # Reset wrists to init
                self.send_angle(3, WRIST1_INIT, force=True)
                self.send_angle(4, WRIST2_INIT, force=True)

                time.sleep(0.1)
        except Exception:
            pass

        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        style = ttk.Style()
        style.theme_use("clam")
    except Exception:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
