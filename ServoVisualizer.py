"""
Robot Hand Control - Main Entry Point
Configures constants and runs the application.
"""

import tkinter as tk
from tkinter import ttk
import math
from robot_hand_classes import ScrollableFrame, App

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
    {"name": "Pinky",   "bottom_ch": 8, "bottom_init": 150.0,  "top_ch": 4, "top_init": 130.0, "extra_ch": None, "extra_init": None},
    {"name": "Ring",    "bottom_ch": 7, "bottom_init": 55.0, "top_ch": 3, "top_init": 140.0,  "extra_ch": None, "extra_init": None},
    {"name": "Middle",  "bottom_ch": 2, "bottom_init": 30.0, "top_ch": 1, "top_init": 140.0, "extra_ch": None, "extra_init": None},
    {"name": "Pointer", "bottom_ch": 5, "bottom_init": 145.0,  "top_ch": 0, "top_init": 40.0, "extra_ch": None, "extra_init": None},
    {"name": "Thumb",   "bottom_ch": 9, "bottom_init": 60.0, "top_ch": 10, "top_init": 60.0, "extra_ch": 6,   "extra_init": 80.0},
]

# --- Animation definitions (uncurled->curled angles) ---
ANIM_TABLE = {
    "Pinky":   {"bottom_uncurled": 150.0,  "bottom_curled": 55.0, "top_uncurled": 130.0, "top_curled": 35.0},
    "Ring":    {"bottom_uncurled": 55.0, "bottom_curled": 140.0,   "top_uncurled": 140.0,  "top_curled": 45.0},
    "Middle":  {"bottom_uncurled": 30.0, "bottom_curled": 120.0,  "top_uncurled": 140.0, "top_curled": 40.0},
    "Pointer": {"bottom_uncurled": 140.0,  "bottom_curled": 50.0, "top_uncurled": 40.0, "top_curled": 125.0},
}

THUMB_TOUCH_POSES = {
    "Pointer": {
        "target": {"bottom": 105.0, "top": 85.0},
        "thumb":  {"bottom": 95.0, "top": 60.0, "extra": 120.0},
    },
    "Middle": {
        "target": {"bottom": 75.0, "top": 100.0},
        "thumb":  {"bottom": 95.0, "top": 55.0, "extra": 130.0},
    },
    "Ring": {
        "target": {"bottom": 100.0, "top": 80.0},
        "thumb":  {"bottom": 95.0, "top": 50.0, "extra": 145.0},
    },
    "Pinky": {
        "target": {"bottom": 105.0, "top": 95.0},
        "thumb":  {"bottom": 95.0, "top": 50.0, "extra": 160.0},
    },
}

DEFAULT_FINGER_IDX = 3  # Pointer

# --- All Animations Sequence Configuration ---
# Simple sequential list of actions. Easy to add/remove/reorder movements!
# 
# Action types:
#   ('wrist', wrist_num, start_angle, end_angle, duration_ms)
#       - wrist_num: 1 or 2
#       - start_angle: starting position (0-270 degrees)
#       - end_angle: ending position (0-270 degrees)
#       - duration_ms: how long the movement takes
#
#   ('delay', duration_ms)
#       - pause for specified milliseconds
#
#   ('finger_wave',)
#       - runs the full 4-finger wave animation
#
#   ('thumb_touch', finger_name)
#       - finger_name: 'Pointer', 'Middle', 'Ring', or 'Pinky'
#
#   ('reset_fingers',)
#       - resets all fingers to their init positions
#
#   ('curl_fingers',)
#       - curls all four fingers (Pinky, Ring, Middle, Pointer) to curled positions
#
#   ('parallel', [list of actions])
#       - runs multiple actions at the same time
#       - example: ('parallel', [('wrist', 1, 135, 90, 1000), ('finger_wave',)])
#
#   ('sequence', [list of actions])
#       - used inside parallel to run actions sequentially
#       - example: ('parallel', [('sequence', [('wrist', 1, 90, 60, 800), ('wrist', 1, 60, 90, 800)]), ('finger_wave',)])
#
ALL_ANIM_SEQUENCE = [
    # Example sequence - customize as you like!
    ('wrist', 1, 135, 145, 50),
    ('wrist', 2, 135, 110, 400),     # Wrist 1: go from 135° to 110° in 800ms
    ('wrist', 2, 110, 160, 600),    
    ('wrist', 2, 160, 110, 600),
    ('wrist', 2, 110, 160, 600),    
    ('wrist', 2, 160, 130, 400),
    ('finger_wave',),                 # Do the full finger wave
    ('parallel', [
        ('sequence', [
            ('wrist', 1, 145, 60, 1200),
            ('wrist', 1, 60, 145, 1200),
        ]),
        ('finger_wave',),             # Do the full finger wave
    ]),
    ('delay', 200),                   # Wait 700ms
    ('thumb_touch', 'Pointer'),       # Thumb touches pointer
    ('delay', 200),                   # Wait 700ms
    ('reset_fingers',),               # Reset all fingers to init positions
    ('delay', 200),                   # Wait 600ms
    ('thumb_touch', 'Middle'),        # Thumb touches middle
    ('delay', 200),  
    ('reset_fingers',),               # Reset all fingers to init positions
    ('delay', 200), 
    ('thumb_touch', 'Ring'),          # Thumb tWouches ring
    ('delay', 200),                   # Wait 600ms
    ('reset_fingers',),               # Reset all fingers to init positions
    ('delay', 200),  
    ('thumb_touch', 'Pinky'),         # Thumb toucWhes pinky
    ('delay', 200),                             # Final pause
    ('reset_fingers',),               # Reset all fingers to init positions
    ('delay', 800),                             # Final pause
    ('curl_fingers',),
    ('wrist', 2, 130, 110, 500),     # Wrist 1: go from 135° to 110° in 800ms
    ('wrist', 2, 110, 160, 1000),
    ('wrist', 2, 160, 130, 500),
    ('wrist', 1, 145, 60, 1000),
    ('wrist', 1, 60, 145, 1000),
    ('reset_fingers',),  # Reset all fingers to init positions
]


def ease_in_out(t: float) -> float:
    """Cosine ease-in-out: smooth wave-like motion, t in [0,1]."""
    t = max(0.0, min(1.0, t))
    return 0.5 - 0.5 * math.cos(math.pi * t)


def main():
    """Main entry point for the application."""
    root = tk.Tk()
    try:
        style = ttk.Style()
        style.theme_use("clam")
    except Exception:
        pass
    
    # Prepare configuration dictionary to pass to App
    config = {
        'FINGERS': FINGERS,
        'ANIM_TABLE': ANIM_TABLE,
        'THUMB_TOUCH_POSES': THUMB_TOUCH_POSES,
        'WRIST1_CH': WRIST1_CH,
        'WRIST2_CH': WRIST2_CH,
        'WRIST1_INIT': WRIST1_INIT,
        'WRIST2_INIT': WRIST2_INIT,
        'WRIST_RANGE': WRIST_RANGE,
        'ALL_FINGERS_NAME': ALL_FINGERS_NAME,
        'WAVE_ORDER': WAVE_ORDER,
        'DEFAULT_FINGER_IDX': DEFAULT_FINGER_IDX,
        'SEND_INTERVAL_SEC': SEND_INTERVAL_SEC,
        'BAUD': BAUD,
        'ease_in_out': ease_in_out,
        'ALL_ANIM_SEQUENCE': ALL_ANIM_SEQUENCE
    }
    
    App(root, config)
    root.mainloop()


if __name__ == "__main__":
    main()
