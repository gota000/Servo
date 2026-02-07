import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time

BAUD = 115200
SEND_INTERVAL_SEC = 0.03  # throttle

# Display-only reference ranges (from your calibration)
TOP_RAW_RANGE = (650, 910)     # top joint raw range (curl..open)
BOT_RAW_RANGE = (110, 385)     # bottom joint raw range (open..curl)

# Servo SAFE ranges (same as Arduino)
S0_MIN, S0_MAX = 155.0, 210.0
S1_MIN, S1_MAX = 90.0, 145.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Robot Finger PID (RAW pot setpoints)")
        self.root.geometry("760x580")

        self.ser = None
        self.last_send_time = 0.0

        # Latest telemetry
        self.raw0 = None
        self.raw1 = None
        self.sp0 = None
        self.sp1 = None
        self.cmd0 = None
        self.cmd1 = None
        self.en0 = 0
        self.en1 = 0

        # ---------- Connection ----------
        top = ttk.Frame(root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Serial Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=32, state="readonly")
        self.port_combo.pack(side="left", padx=8)

        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=8)

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side="left", padx=10)

        # ---------- Telemetry ----------
        tele = ttk.LabelFrame(root, text="Telemetry", padding=10)
        tele.pack(fill="x", padx=10, pady=(5, 10))

        self.tele_var = tk.StringVar(value="Waiting for POT telemetry...")
        ttk.Label(tele, textvariable=self.tele_var, font=("Segoe UI", 10, "bold")).pack(anchor="w")

        # ---------- Joint Frames ----------
        body = ttk.Frame(root, padding=10)
        body.pack(fill="both", expand=True)

        self.j0 = self.make_joint_frame(
            body, joint=0, title="Top Joint (Channel 0)",
            raw_hint=TOP_RAW_RANGE, safe_min=S0_MIN, safe_max=S0_MAX,
            default_kp="0.20", default_ki="0.00", default_kd="0.00"
        )

        self.j1 = self.make_joint_frame(
            body, joint=1, title="Bottom Joint (Channel 1)",
            raw_hint=BOT_RAW_RANGE, safe_min=S1_MIN, safe_max=S1_MAX,
            default_kp="0.50", default_ki="0.30", default_kd="0.02"
        )

        ttk.Label(
            root,
            text="Serial Commands: PID,idx,kp,ki,kd | SP,idx,raw | EN,idx,0/1 | MAN,idx,deg",
            foreground="#444"
        ).pack(anchor="w", padx=12, pady=6)

        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def make_joint_frame(self, parent, joint: int, title: str, raw_hint, safe_min: float, safe_max: float,
                         default_kp="0.20", default_ki="0.00", default_kd="0.00"):
        f = ttk.LabelFrame(parent, text=title, padding=10)
        f.pack(fill="x", pady=8)

        ttk.Label(
            f,
            text=f"Raw hint range: {raw_hint[0]} .. {raw_hint[1]}   |   Safe servo: {safe_min:.1f} .. {safe_max:.1f}",
        ).pack(anchor="w")

        # Gains row
        gains = ttk.Frame(f)
        gains.pack(fill="x", pady=(8, 2))

        ttk.Label(gains, text="Kp").pack(side="left")
        kp_var = tk.StringVar(value=default_kp)
        ttk.Entry(gains, textvariable=kp_var, width=8).pack(side="left", padx=(4, 12))

        ttk.Label(gains, text="Ki").pack(side="left")
        ki_var = tk.StringVar(value=default_ki)
        ttk.Entry(gains, textvariable=ki_var, width=8).pack(side="left", padx=(4, 12))

        ttk.Label(gains, text="Kd").pack(side="left")
        kd_var = tk.StringVar(value=default_kd)
        ttk.Entry(gains, textvariable=kd_var, width=8).pack(side="left", padx=(4, 12))

        def send_gains():
            try:
                kp = float(kp_var.get())
                ki = float(ki_var.get())
                kd = float(kd_var.get())
            except ValueError:
                messagebox.showerror("Bad gains", "Kp/Ki/Kd must be numbers.")
                return
            self.send_line(f"PID,{joint},{kp},{ki},{kd}")

        ttk.Button(gains, text="Send gains", command=send_gains).pack(side="left", padx=8)

        # Setpoint row
        sp = ttk.Frame(f)
        sp.pack(fill="x", pady=(10, 2))

        ttk.Label(sp, text="Target raw:").pack(side="left")
        sp_var = tk.StringVar(value="")
        sp_entry = ttk.Entry(sp, textvariable=sp_var, width=10)
        sp_entry.pack(side="left", padx=(6, 10))

        def send_sp():
            try:
                raw = int(float(sp_var.get()))
            except ValueError:
                messagebox.showerror("Bad setpoint", "Target raw must be a number.")
                return
            self.send_line(f"SP,{joint},{raw}")

        ttk.Button(sp, text="Send target", command=send_sp).pack(side="left")

        def set_target_to_current():
            raw = self.raw0 if joint == 0 else self.raw1
            if raw is None:
                return
            sp_var.set(str(raw))
            self.send_line(f"SP,{joint},{raw}")

        ttk.Button(sp, text="Set target = current raw", command=set_target_to_current).pack(side="left", padx=10)

        def enable_pid():
            self.send_line(f"EN,{joint},1")

        def disable_pid():
            self.send_line(f"EN,{joint},0")

        ttk.Button(sp, text="Enable PID", command=enable_pid).pack(side="left", padx=(10, 4))
        ttk.Button(sp, text="Disable PID", command=disable_pid).pack(side="left")

        # Manual slider row (manual override disables PID on Arduino)
        ttk.Label(f, text="Manual servo cmd (disables PID for this joint):").pack(anchor="w", pady=(10, 0))
        slider = ttk.Scale(
            f, from_=safe_min, to=safe_max, orient="horizontal",
            command=lambda v: self.on_manual(joint, float(v))
        )
        slider.pack(fill="x", pady=6)
        slider.set(155.0 if joint == 0 else 90.0)

        return {"kp_var": kp_var, "ki_var": ki_var, "kd_var": kd_var, "sp_var": sp_var, "slider": slider}

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
            time.sleep(1.5)  # allow board reset
            self.status_var.set(f"Connected: {port}")
            self.connect_btn.config(text="Disconnect")
            self.root.after(20, self.poll_serial)
        except Exception as e:
            self.ser = None
            messagebox.showerror("Connect failed", str(e))
            self.status_var.set("Disconnected")
            self.connect_btn.config(text="Connect")

    def disconnect(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.status_var.set("Disconnected")
        self.connect_btn.config(text="Connect")

    def send_line(self, msg: str):
        if not (self.ser and self.ser.is_open):
            return
        now = time.time()
        if now - self.last_send_time < SEND_INTERVAL_SEC:
            return
        try:
            self.ser.write((msg.strip() + "\n").encode("utf-8"))
            self.last_send_time = now
        except Exception as e:
            messagebox.showerror("Serial write failed", str(e))
            self.disconnect()

    def on_manual(self, joint: int, deg: float):
        self.send_line(f"MAN,{joint},{deg:.1f}")

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
        # Expected:
        # POT,raw0,raw1,sp0,sp1,cmd0,cmd1,en0,en1
        if not line.startswith("POT,"):
            return
        parts = line.split(",")
        if len(parts) != 9:
            return

        _, raw0, raw1, sp0, sp1, cmd0, cmd1, en0, en1 = parts
        try:
            self.raw0 = int(float(raw0))
            self.raw1 = int(float(raw1))
            self.sp0 = int(float(sp0))
            self.sp1 = int(float(sp1))
            self.cmd0 = float(cmd0)
            self.cmd1 = float(cmd1)
            self.en0 = int(en0)
            self.en1 = int(en1)
        except ValueError:
            return

        self.tele_var.set(
            f"Top: raw={self.raw0:4d}  sp={self.sp0:4d}  cmd={self.cmd0:6.1f}  PID={'ON' if self.en0 else 'OFF'}\n"
            f"Bot: raw={self.raw1:4d}  sp={self.sp1:4d}  cmd={self.cmd1:6.1f}  PID={'ON' if self.en1 else 'OFF'}"
        )

    def on_close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b"EN,0,0\n")
                self.ser.write(b"EN,1,0\n")
                time.sleep(0.05)
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
