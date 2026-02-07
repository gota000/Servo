#!/usr/bin/env python3
"""
Servo control with GUI slider for PCA9685 on Raspberry Pi
SMRAZA 45kg Digital Servo SC55-NA (270 degrees)
Requires X11 forwarding (XLaunch on Windows)
"""

import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import tkinter as tk
from tkinter import ttk

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize PCA9685
pca = PCA9685(i2c)
pca.frequency = 333  # Standard servo frequency

# Servo channels
SERVO_CHANNEL_0 = 0
SERVO_CHANNEL_1 = 1
SERVO_CHANNEL_4 = 4

# Create servo objects
# Adjust min_pulse / max_pulse if needed based on how far it actually moves
servo_0 = servo.Servo(
    pca.channels[SERVO_CHANNEL_0],
    min_pulse=500,    # microseconds
    max_pulse=2500,   # microseconds
    actuation_range=270  # degrees for your 270° servo
)

servo_1 = servo.Servo(
    pca.channels[SERVO_CHANNEL_1],
    min_pulse=500,    # microseconds
    max_pulse=2500,   # microseconds
    actuation_range=180  # degrees for your 180° servo
)

servo_4 = servo.Servo(
    pca.channels[SERVO_CHANNEL_4],
    min_pulse=500,    # microseconds
    max_pulse=2500,   # microseconds
    actuation_range=270  # degrees for your 270° servo
)

class ServoControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Triple Servo Control")
        self.root.geometry("450x520")
        
        # Set initial positions to center
        servo_0.angle = 135
        servo_1.angle = 90
        servo_4.angle = 135
        
        # Title label
        title_label = tk.Label(root, text="Triple Servo Position Control", font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Servo 0 controls
        servo0_frame = tk.Frame(root)
        servo0_frame.pack(pady=10, padx=20, fill=tk.X)
        
        tk.Label(servo0_frame, text="Servo 0 (Channel 0)", font=("Arial", 11, "bold")).pack()
        self.angle_label_0 = tk.Label(servo0_frame, text="Angle: 135°", font=("Arial", 10))
        self.angle_label_0.pack(pady=5)
        
        self.slider_0 = ttk.Scale(
            servo0_frame,
            from_=0,
            to=270,
            orient=tk.HORIZONTAL,
            length=350,
            command=self.update_servo_0
        )
        self.slider_0.set(135)
        self.slider_0.pack(pady=5)
        
        labels_frame_0 = tk.Frame(servo0_frame)
        labels_frame_0.pack()
        tk.Label(labels_frame_0, text="0°").pack(side=tk.LEFT, padx=10)
        tk.Label(labels_frame_0, text="270°").pack(side=tk.RIGHT, padx=10)
        
        # Separator
        ttk.Separator(root, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        # Servo 1 controls
        servo1_frame = tk.Frame(root)
        servo1_frame.pack(pady=10, padx=20, fill=tk.X)
        
        tk.Label(servo1_frame, text="Servo 1 (Channel 1) - 180°", font=("Arial", 11, "bold")).pack()
        self.angle_label_1 = tk.Label(servo1_frame, text="Angle: 90°", font=("Arial", 10))
        self.angle_label_1.pack(pady=5)
        
        self.slider_1 = ttk.Scale(
            servo1_frame,
            from_=0,
            to=180,
            orient=tk.HORIZONTAL,
            length=350,
            command=self.update_servo_1
        )
        self.slider_1.set(90)
        self.slider_1.pack(pady=5)
        
        labels_frame_1 = tk.Frame(servo1_frame)
        labels_frame_1.pack()
        tk.Label(labels_frame_1, text="0°").pack(side=tk.LEFT, padx=10)
        tk.Label(labels_frame_1, text="180°").pack(side=tk.RIGHT, padx=10)
        
        # Separator
        ttk.Separator(root, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        # Servo 4 controls
        servo4_frame = tk.Frame(root)
        servo4_frame.pack(pady=10, padx=20, fill=tk.X)
        
        tk.Label(servo4_frame, text="Servo 4 (Channel 4)", font=("Arial", 11, "bold")).pack()
        self.angle_label_4 = tk.Label(servo4_frame, text="Angle: 135°", font=("Arial", 10))
        self.angle_label_4.pack(pady=5)
        
        self.slider_4 = ttk.Scale(
            servo4_frame,
            from_=0,
            to=270,
            orient=tk.HORIZONTAL,
            length=350,
            command=self.update_servo_4
        )
        self.slider_4.set(135)
        self.slider_4.pack(pady=5)
        
        labels_frame_4 = tk.Frame(servo4_frame)
        labels_frame_4.pack()
        tk.Label(labels_frame_4, text="0°").pack(side=tk.LEFT, padx=10)
        tk.Label(labels_frame_4, text="270°").pack(side=tk.RIGHT, padx=10)
        
    def update_servo_0(self, value):
        """Update servo 0 position when slider moves"""
        angle = float(value)
        servo_0.angle = angle
        self.angle_label_0.config(text=f"Angle: {angle:.1f}°")
    
    def update_servo_1(self, value):
        """Update servo 1 position when slider moves"""
        angle = float(value)
        servo_1.angle = angle
        self.angle_label_1.config(text=f"Angle: {angle:.1f}°")
    
    def update_servo_4(self, value):
        """Update servo 4 position when slider moves"""
        angle = float(value)
        servo_4.angle = angle
        self.angle_label_4.config(text=f"Angle: {angle:.1f}°")
    
    def cleanup(self):
        """Return servos to center on exit"""
        servo_0.angle = 135
        servo_1.angle = 90
        servo_4.angle = 135
        time.sleep(0.2)
        pca.deinit()

def main():
    root = tk.Tk()
    app = ServoControlGUI(root)
    
    def on_closing():
        app.cleanup()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
