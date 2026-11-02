import cv2
import mediapipe as mp
import numpy as np
import math
import time
import os
import sys
import serial

# ------------------------------------------------------
#                Serial Setup
# ------------------------------------------------------
arduino = serial.Serial("COM6", 115200, timeout=1)
time.sleep(2)

# ------------------------------------------------------
#                Mediapipe Setup
# ------------------------------------------------------
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    min_detection_confidence=0.7,
    min_tracking_confidence=0.6
)
mp_draw = mp.solutions.drawing_utils

# ------------------------------------------------------
#                Helper functions
# ------------------------------------------------------
def get_point(lm, w, h):
    return int(lm.x * w), int(lm.y * h)

def angle(a, b, c):
    return abs(math.degrees(
        math.atan2(c[1]-b[1], c[0]-b[0]) -
        math.atan2(a[1]-b[1], a[0]-b[0])
    ))

def bend_percent(raw_angle):
    raw_angle = max(30, min(180, raw_angle))
    return (180 - raw_angle) / (180 - 30) * 100

# ------------------------------------------------------
#                Smoothing
# ------------------------------------------------------
SMOOTH_ALPHA = 0.2
smoothed_bend = [0.0, 0.0]   # index, middle

# ------------------------------------------------------
#                Terminal Dashboard Printer
# ------------------------------------------------------
def clear_screen():
    if os.name == "nt":
        os.system("cls")
    else:
        sys.stdout.write("\033[2J\033[H")

def print_dashboard(angles_deg, bends):
    clear_screen()
    print("=== REALTIME FINGER / SERVO ===")
    print(f"Index : {angles_deg[0]:4d}°   ({bends[0]:5.1f}% bend)")
    print(f"Middle: {angles_deg[1]:4d}°   ({bends[1]:5.1f}% bend)")
    print("--------------------------------")

# ------------------------------------------------------
#                Main Loop
# ------------------------------------------------------
cap = cv2.VideoCapture(0)

FINGER_IDS = [
    (5, 6, 8),    # Index
    (9, 10, 12)   # Middle
]

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    bend_values = [0.0, 0.0]

    if result.multi_hand_landmarks:
        lm = result.multi_hand_landmarks[0]

        for i, (a_id, b_id, c_id) in enumerate(FINGER_IDS):
            a = get_point(lm.landmark[a_id], w, h)
            b = get_point(lm.landmark[b_id], w, h)
            c = get_point(lm.landmark[c_id], w, h)

            raw_ang = angle(a, b, c)
            pct = bend_percent(raw_ang)

            smoothed_bend[i] = (
                SMOOTH_ALPHA * pct +
                (1 - SMOOTH_ALPHA) * smoothed_bend[i]
            )
            bend_values[i] = smoothed_bend[i]

    # -------------------------
    #   Convert bend → angle (0–180)
    # -------------------------
    servo_angles = [
        int(np.clip(b / 100 * 180, 0, 180))
        for b in bend_values
    ]

    # -------------------------
    #   Send angles to Arduino
    # -------------------------
    line = f"{servo_angles[0]} {servo_angles[1]}\n"
    arduino.write(line.encode())

    # -------------------------
    #   Print dashboard
    # -------------------------
    print_dashboard(servo_angles, bend_values)

    # -------------------------
    #   Show camera window
    # -------------------------
    cv2.imshow("Hand", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
