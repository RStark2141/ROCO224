import os
import cv2
import mediapipe as mp
import time
import numpy as np
import socket

# Silence TensorFlow logs so we only see our own messages
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# Initialize MediaPipe Face Mesh for lip detection
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    min_detection_confidence=0.5,   # require at least 50% confidence
    min_tracking_confidence=0.5     # same for tracking
)

# How far apart lips have to be (relative) to count as "open"
open_threshold = 0.1

#UDP setup to talk to ESP32
ESP32_IP = "192.168.3.68"
UDP_PORT = 12345
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_udp(cmd: str):
    """Send a short string command over UDP to the ESP32."""
    try:
        udp_sock.sendto(cmd.encode('utf-8'), (ESP32_IP, UDP_PORT))
        print(f"📡 Sent UDP '{cmd}'")
    except Exception as e:
        print("Error sending UDP:", e)

# Capture from local laptop camera first, then fallback to ESP32-CAM stream
STREAM_URL = "http://192.168.3.68:81/stream"
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Local camera failed → trying ESP32-CAM stream…")
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open stream at {STREAM_URL}")

start_time = time.time()
first_frame = False
NO_FRAME_TIMEOUT = 60    # seconds before we re-init stream if no frames
last_frame_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        # If we haven't seen a frame in a while, reset the stream
        elapsed = time.time() - last_frame_time
        if elapsed > NO_FRAME_TIMEOUT:
            print("No frames for 60s, reinitializing capture…")
            cap.release()
            cap = cv2.VideoCapture(STREAM_URL)
            last_frame_time = time.time()
        # Show a blank waiting screen with a timer
        blank = np.zeros((480,640,3), dtype=np.uint8)
        cv2.putText(blank, f"Waiting… {elapsed:.1f}s",
                    (50,240), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0,0,255), 2)
        cv2.imshow('Lip Detect', blank)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    # We got a frame—update last_frame_time and maybe note first arrival
    last_frame_time = time.time()
    if not first_frame:
        print(f"First frame after {last_frame_time-start_time:.1f}s")
        first_frame = True

    # Convert image to RGB for MediaPipe processing
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb)

    mouth_state = "Unknown"
    color = (0,255,255)  # default yellowish highlight
    if results.multi_face_landmarks:
        lm = results.multi_face_landmarks[0].landmark
        ul = lm[13]; ll = lm[14]; lc = lm[61]; rc = lm[291]
        # Compute ratio of vertical lip opening to mouth width
        ratio = abs(ul.y-ll.y)/abs(lc.x-rc.x) if abs(lc.x-rc.x)>0 else 0

        # Decide open vs closed
        if ratio > open_threshold:
            mouth_state = "Open"; color=(0,0,255)
        else:
            mouth_state = "Closed"; color=(255,0,0)
        print(f"Mouth {mouth_state} (ratio {ratio:.3f})")
        send_udp(mouth_state)  # Send state to ESP32

        # Draw lip landmarks for visual feedback
        for idx in [61,146,91,181,84,17,314,405,321,375,78,191,80,81,82,
                    13,312,311,310,415,308,324,318,402,317,14,87,178,88]:
            x = int(lm[idx].x*frame.shape[1])
            y = int(lm[idx].y*frame.shape[0])
            cv2.circle(frame,(x,y),2,(0,255,0),-1)

    # Overlay status text on the frame
    cv2.putText(frame, f"Mouth: {mouth_state}", (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.imshow('Lip Detect', frame)
    if cv2.waitKey(1)&0xFF==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
