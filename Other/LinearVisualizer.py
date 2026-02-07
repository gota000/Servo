import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

BAUD = 115200

def list_serial_ports():
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append(p.device)
    return ports

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Linear Actuator Control (IN1–IN4)")
        self.geometry("520x320")

        self.ser = None
        self.reader_thread = None
        self.reader_running = False

        self.port_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Disconnected")

        self._build_ui()
        self._refresh_ports()

    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Serial Port:").pack(side="left")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=20, state="readonly")
        self.port_combo.pack(side="left", padx=8)

        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")
        ttk.Button(top, text="Connect", command=self.connect).pack(side="left", padx=6)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side="left")

        ttk.Label(self, textvariable=self.status_var, padding=(10, 5)).pack(anchor="w")

        body = ttk.Frame(self, padding=10)
        body.pack(fill="both", expand=True)

        # Actuator 1 controls
        a1 = ttk.LabelFrame(body, text="Actuator 1", padding=10)
        a1.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        # Actuator 2 controls
        a2 = ttk.LabelFrame(body, text="Actuator 2", padding=10)
        a2.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        # Buttons with press-and-hold behavior:
        # Press: extend/retract, Release: stop
        self._make_hold_button(a1, "Extend (hold)", press_cmd="A1E", release_cmd="A1S").pack(fill="x", pady=4)
        self._make_hold_button(a1, "Retract (hold)", press_cmd="A1R", release_cmd="A1S").pack(fill="x", pady=4)
        ttk.Button(a1, text="Stop", command=lambda: self.send("A1S")).pack(fill="x", pady=4)

        self._make_hold_button(a2, "Extend (hold)", press_cmd="A2E", release_cmd="A2S").pack(fill="x", pady=4)
        self._make_hold_button(a2, "Retract (hold)", press_cmd="A2R", release_cmd="A2S").pack(fill="x", pady=4)
        ttk.Button(a2, text="Stop", command=lambda: self.send("A2S")).pack(fill="x", pady=4)

        bottom = ttk.Frame(self, padding=10)
        bottom.pack(fill="x")

        ttk.Button(bottom, text="STOP ALL", command=lambda: self.send("STOP")).pack(side="left")

        self.log = tk.Text(bottom, height=6, wrap="word")
        self.log.pack(side="left", fill="both", expand=True, padx=10)
        self._log_line("Tip: Hold Extend/Retract to move; release to stop.")

    def _make_hold_button(self, parent, text, press_cmd, release_cmd):
        btn = ttk.Button(parent, text=text)
        btn.bind("<ButtonPress-1>", lambda e: self.send(press_cmd))
        btn.bind("<ButtonRelease-1>", lambda e: self.send(release_cmd))
        return btn

    def _refresh_ports(self):
        ports = list_serial_ports()
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        if not ports:
            self.port_var.set("")

    def connect(self):
        if self.ser:
            return
        port = self.port_var.get()
        if not port:
            messagebox.showerror("No port", "No serial port selected.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.1)
            time.sleep(0.4)  # allow board to reset/print READY
            self.status_var.set(f"Connected: {port} @ {BAUD}")
            self._start_reader()
            self.send("STOP")
        except Exception as e:
            self.ser = None
            messagebox.showerror("Connect failed", str(e))

    def disconnect(self):
        self._stop_reader()
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        self.ser = None
        self.status_var.set("Disconnected")

    def send(self, cmd):
        if not self.ser:
            self._log_line(f"(not sent, disconnected) {cmd}")
            return
        try:
            self.ser.write((cmd.strip() + "\n").encode("utf-8"))
            self._log_line(f"> {cmd}")
        except Exception as e:
            self._log_line(f"! send error: {e}")

    def _start_reader(self):
        self.reader_running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

    def _stop_reader(self):
        self.reader_running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=0.3)
        self.reader_thread = None

    def _reader_loop(self):
        while self.reader_running and self.ser:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.after(0, lambda l=line: self._log_line(f"< {l}"))
            except:
                pass
            time.sleep(0.02)

    def _log_line(self, s):
        self.log.insert("end", s + "\n")
        self.log.see("end")

if __name__ == "__main__":
    app = App()
    app.mainloop()
