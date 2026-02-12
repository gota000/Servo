import os

# Suppress TensorFlow logs
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"

# Suppress absl logs (used by MediaPipe)
os.environ["GLOG_minloglevel"] = "3"
os.environ["ABSL_MIN_LOG_LEVEL"] = "3"

import cv2
import socket
import json
import threading
import math
import tkinter as tk
from tkinter import messagebox
import mediapipe as mp

ALPHA_VALUE = 0.3  # Smoothing filter (lower = faster response, higher = smoother but slower)
CURL_CALIBRATION_FILE = "curl_calibration_2dof.json"


class HandTracker:
    def __init__(self, on_scaled_angles):
        self.on_scaled_angles = on_scaled_angles
        self.finger_mins = {}  # Will store min angles for both joints
        self.finger_maxs = {}  # Will store max angles for both joints
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Define landmark indices for each finger
        # Each finger has 4 landmarks: MCP, PIP, DIP, TIP
        self.fingers = {
            "thumb": [1, 2, 3, 4],
            "index": [5, 6, 7, 8],
            "middle": [9, 10, 11, 12],
            "ring": [13, 14, 15, 16],
            "pinky": [17, 18, 19, 20]
        }
        
        # For calculating angles, we need to define which landmarks form each joint
        # Bottom joint (proximal): angle at PIP - formed by MCP, PIP, DIP
        # Top joint (distal): angle at DIP - formed by PIP, DIP, TIP
        self.joint_definitions = {
            "thumb": {
                "bottom": [1, 2, 3],  # CMC to MCP joint
                "top": [2, 3, 4]      # MCP to IP joint
            },
            "index": {
                "bottom": [5, 6, 7],  # MCP to PIP joint
                "top": [6, 7, 8]      # PIP to DIP joint
            },
            "middle": {
                "bottom": [9, 10, 11],
                "top": [10, 11, 12]
            },
            "ring": {
                "bottom": [13, 14, 15],
                "top": [14, 15, 16]
            },
            "pinky": {
                "bottom": [17, 18, 19],
                "top": [18, 19, 20]
            }
        }
        
        self.load_curl_calibration()
        self.tracking_active = False

    def load_curl_calibration(self):
        if os.path.exists(CURL_CALIBRATION_FILE):
            try:
                with open(CURL_CALIBRATION_FILE, "r") as f:
                    data = json.load(f)
                    self.finger_mins = data.get("min", {})
                    self.finger_maxs = data.get("max", {})
                print(f"[INFO] Loaded curl calibration from {CURL_CALIBRATION_FILE}")
            except Exception as e:
                print(f"[ERROR] Failed to load curl calibration: {e}")

    def save_curl_calibration(self):
        data = {
            "min": self.finger_mins,
            "max": self.finger_maxs
        }
        try:
            with open(CURL_CALIBRATION_FILE, "w") as f:
                json.dump(data, f, indent=2)
            print(f"[INFO] Saved curl calibration to {CURL_CALIBRATION_FILE}")
        except Exception as e:
            print(f"[ERROR] Failed to save curl calibration: {e}")

    def calculate_angle(self, a, b, c):
        """Calculate angle at point b formed by points a-b-c"""
        ab = [b[i] - a[i] for i in range(3)]
        cb = [b[i] - c[i] for i in range(3)]
        dot = sum(ab[i] * cb[i] for i in range(3))
        mag_ab = math.sqrt(sum(x ** 2 for x in ab))
        mag_cb = math.sqrt(sum(x ** 2 for x in cb))
        if mag_ab * mag_cb == 0: 
            return 0
        cos_angle = max(-1, min(1, dot / (mag_ab * mag_cb)))  # Clamp to avoid domain errors
        angle = math.acos(cos_angle)
        return math.degrees(angle)

    def scale_angle(self, finger_name, joint_name, raw):
        """Scale raw angle to 0-180 range based on calibration"""
        key = f"{finger_name}_{joint_name}"
        min_a = self.finger_mins.get(key, 30)
        max_a = self.finger_maxs.get(key, 160)
        if max_a - min_a == 0: 
            return 90
        # INVERTED: when raw is max (open), returns 0; when raw is min (closed), returns 180
        return round(max(0, min(1, (max_a - raw) / (max_a - min_a))) * 180)

    def calibrate(self):
        def capture_position(position):
            root = tk.Tk()
            root.withdraw()
            messagebox.showinfo(f"{position} Calibration", 
                              f"Please {position.lower()} your hand completely and click OK")
            root.destroy()
            
            cap = cv2.VideoCapture(0)
            frames_captured = []
            
            # Capture multiple frames for better calibration
            with self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7) as hands:
                for _ in range(15):  # Capture 15 frames
                    ret, frame = cap.read()
                    if not ret:
                        continue
                    frame = cv2.flip(frame, 1)
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    result = hands.process(rgb)
                    
                    if result.multi_hand_landmarks:
                        lm = [(lm.x, lm.y, lm.z) for lm in result.multi_hand_landmarks[0].landmark]
                        frames_captured.append(lm)
                    
                    cv2.imshow("Calibration", frame)
                    cv2.waitKey(50)
            
            cap.release()
            cv2.destroyAllWindows()

            if not frames_captured:
                print("No hand detected during calibration.")
                return None

            # Average the angles from all captured frames
            snapshots = []
            for lm in frames_captured:
                snapshot = {}
                for name, joints in self.joint_definitions.items():
                    # Bottom joint angle
                    a, b, c = lm[joints["bottom"][0]], lm[joints["bottom"][1]], lm[joints["bottom"][2]]
                    snapshot[f"{name}_bottom"] = self.calculate_angle(a, b, c)
                    
                    # Top joint angle
                    a, b, c = lm[joints["top"][0]], lm[joints["top"][1]], lm[joints["top"][2]]
                    snapshot[f"{name}_top"] = self.calculate_angle(a, b, c)
                snapshots.append(snapshot)
            
            # Average all snapshots
            averaged = {}
            for key in snapshots[0].keys():
                averaged[key] = sum(s[key] for s in snapshots) / len(snapshots)
            
            return averaged

        print("[INFO] Starting calibration...")
        maxs = capture_position("OPEN")
        mins = capture_position("CLOSED")
        
        if maxs and mins:
            self.finger_maxs = maxs
            self.finger_mins = mins
            self.save_curl_calibration()
            print("✅ Calibration complete!")
            print(f"[DEBUG] Open position angles: {maxs}")
            print(f"[DEBUG] Closed position angles: {mins}")
        else:
            print("❌ Calibration failed.")

    def run_tracking(self):
        if not self.finger_mins or not self.finger_maxs:
            print("[ERROR] Please calibrate first!")
            return
        
        self.tracking_active = True
        cap = cv2.VideoCapture(0)
        
        with self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7) as hands:
            while cap.isOpened() and self.tracking_active:
                ret, frame = cap.read()
                if not ret:
                    break
                
                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                result = hands.process(rgb)

                if result.multi_hand_landmarks:
                    landmarks = [(lm.x, lm.y, lm.z) for lm in result.multi_hand_landmarks[0].landmark]
                    scaled = {}
                    
                    # Calculate angles for both joints of each finger
                    for name, joints in self.joint_definitions.items():
                        # Bottom joint
                        a, b, c = landmarks[joints["bottom"][0]], landmarks[joints["bottom"][1]], landmarks[joints["bottom"][2]]
                        raw_angle_bottom = self.calculate_angle(a, b, c)
                        scaled[f"{name}_bottom"] = self.scale_angle(name, "bottom", raw_angle_bottom)
                        
                        # Top joint
                        a, b, c = landmarks[joints["top"][0]], landmarks[joints["top"][1]], landmarks[joints["top"][2]]
                        raw_angle_top = self.calculate_angle(a, b, c)
                        scaled[f"{name}_top"] = self.scale_angle(name, "top", raw_angle_top)
                    
                    # Send scaled angles to callback
                    self.on_scaled_angles(scaled)
                    
                    # Draw hand landmarks on frame
                    self.mp_drawing.draw_landmarks(
                        frame, 
                        result.multi_hand_landmarks[0],
                        self.mp_hands.HAND_CONNECTIONS
                    )

                cv2.imshow("Hand Tracking (Press 'q' to stop)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.tracking_active = False
        cap.release()
        cv2.destroyAllWindows()
    
    def stop_tracking(self):
        self.tracking_active = False


class RobotHandClient:
    def __init__(self, ip="172.20.10.2", port=9999):
        self.addr = (ip, port)
        self.sock = None
        self.connected = False
        self.connect()

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(self.addr)
            self.connected = True
            print(f"[INFO] ✅ Connected to Raspberry Pi at {self.addr[0]}:{self.addr[1]}")
        except Exception as e:
            print(f"[ERROR] ❌ Connection failed: {e}")
            self.connected = False

    def send_angles(self, angles):
        if not self.connected:
            print("[ERROR] Not connected to robot. Cannot send angles.")
            return
        
        try:
            data = json.dumps(angles) + "\n"  # Add newline as message delimiter
            self.sock.sendall(data.encode())
        except Exception as e:
            print(f"[ERROR] Sending angles failed: {e}")
            self.connected = False


class FilteredSender:
    def __init__(self, send_callback, alpha=ALPHA_VALUE):
        self.send_callback = send_callback
        self.alpha = alpha
        self.filtered_angles = {}

    def send_filtered_angles(self, new_angles):
        smoothed = {}
        for joint, angle in new_angles.items():
            if joint not in self.filtered_angles:
                self.filtered_angles[joint] = angle
            else:
                # Apply low-pass filter: new = alpha * old + (1 - alpha) * new
                self.filtered_angles[joint] = (
                    self.alpha * self.filtered_angles[joint] + (1 - self.alpha) * angle
                )
            smoothed[joint] = round(self.filtered_angles[joint])
        
        print(f"[DEBUG] Sending angles to servos: {smoothed}")
        self.send_callback(smoothed)


class RobotHandApp:
    def __init__(self):
        self.client = RobotHandClient()
        self.filtered_sender = FilteredSender(self.client.send_angles)
        self.tracker = HandTracker(self.filtered_sender.send_filtered_angles)

    def start(self):
        root = tk.Tk()
        root.title("Robot Hand Controller - 2 DOF")
        root.geometry("300x200")
        
        # Title label
        tk.Label(root, text="Robot Hand Control", font=("Arial", 14, "bold")).pack(pady=10)
        
        # Calibration button
        tk.Button(
            root, 
            text="Calibrate Hand", 
            font=("Arial", 12),
            bg="#4CAF50",
            fg="white",
            command=self.tracker.calibrate,
            width=20,
            height=2
        ).pack(pady=5)
        
        # Start tracking button
        tk.Button(
            root, 
            text="Start Tracking", 
            font=("Arial", 12),
            bg="#2196F3",
            fg="white",
            command=lambda: threading.Thread(target=self.tracker.run_tracking, daemon=True).start(),
            width=20,
            height=2
        ).pack(pady=5)
        
        # Stop tracking button
        tk.Button(
            root, 
            text="Stop Tracking", 
            font=("Arial", 12),
            bg="#f44336",
            fg="white",
            command=self.tracker.stop_tracking,
            width=20,
            height=2
        ).pack(pady=5)
        
        root.mainloop()


if __name__ == "__main__":
    RobotHandApp().start()
