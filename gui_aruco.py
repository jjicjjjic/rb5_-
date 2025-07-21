import tkinter as tk
from tkinter import ttk
import socket
import threading
import json
import cv2
import numpy as np


HOST = '0.0.0.0'
PORT = 12345

client_conn = None
labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

# ArUco 설정
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
MARKER_LENGTH = 0.03  # 30mm 마커 [미터 단위]

# 카메라 보정값 (예시용, 실제 보정값으로 교체 필요)
camera_matrix = np.array([[600, 0, 320],
                          [0, 600, 240],
                          [0,   0,   1]], dtype=np.float64)
dist_coeffs = np.zeros((5, 1))  # 왜곡 없음 가정



root = tk.Tk()
root.title("🤖 Robot Control Server GUI")
# root.geometry("420x450")  ← ❌ 창 크기 고정하지 않음!
root.resizable(False, False)

style = ttk.Style()
style.configure("TLabel", font=("Arial", 11))
style.configure("TButton", font=("Arial", 11))
style.configure("Feedback.TLabel", font=("Arial", 11, "bold"), foreground="blue")


# 피드백 구역
feedback_frame = ttk.LabelFrame(root, text="📩 클라이언트 피드백", padding=10)
feedback_frame.pack(fill="x", padx=10, pady=5)

feedback_labels = {}
for i, label in enumerate(labels):
    ttk.Label(feedback_frame, text=label).grid(row=i, column=0, sticky="e", pady=2)
    lbl = ttk.Label(feedback_frame, text="--", style="Feedback.TLabel")
    lbl.grid(row=i, column=1, sticky="w", padx=5)
    feedback_labels[label] = lbl

# 버튼 프레임
button_frame = ttk.LabelFrame(root, text="🕹️ 제어", padding=10)
button_frame.pack(padx=10, pady=10, fill="x")

def send_stop():
    if client_conn:
    	msg = json.dumps(values).encode() + b'\n'
        client_conn.sendall(b's')
        print("[SERVER] Sent STOP signal ('s')")

stop_button = ttk.Button(button_frame, text="■ Stop", command=send_stop)
stop_button.grid(row=0, column=1, padx=20, pady=5)

# 서버 수신 스레드
def receive_loop(conn):
    buffer = ""
    while True:
        try:
            data = conn.recv(1024)
            if not data:
                break
            buffer += data.decode()

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line)
                    print("[SERVER] Received:", msg)
                    for k in labels:
                        feedback_labels[k].config(text=str(msg.get(k, '--')))
                except json.JSONDecodeError:
                    print("[WARN] Invalid JSON received:", line)
        except Exception as e:
            print(f"[ERROR] {e}")
            break
    conn.close()



def aruco_thread():
    global client_conn

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Camera open failed")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, camera_matrix, dist_coeffs)

            for rvec, tvec in zip(rvecs, tvecs):
                # 위치 [m] → [mm]
                x, y, z = (tvec[0] * 1000.0).tolist()

                # 자세 (Rodrigues → Euler)
                R, _ = cv2.Rodrigues(rvec)
                sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
                singular = sy < 1e-6
                if not singular:
                    roll = np.arctan2(R[2,1], R[2,2])
                    pitch = np.arctan2(-R[2,0], sy)
                    yaw = np.arctan2(R[1,0], R[0,0])
                else:
                    roll = np.arctan2(-R[1,2], R[1,1])
                    pitch = np.arctan2(-R[2,0], sy)
                    yaw = 0

                # 라디안 → 도
                roll_deg = np.degrees(roll)
                pitch_deg = np.degrees(pitch)
                yaw_deg = np.degrees(yaw)

                # 값 묶기
                values = {
                    'x': round(x, 1), 'y': round(y, 1), 'z': round(z, 1),
                    'roll': round(roll_deg, 1),
                    'pitch': round(pitch_deg, 1),
                    'yaw': round(yaw_deg, 1)
                }

                # GUI 피드백 업데이트
                for k in labels:
                    feedback_labels[k].config(text=str(values[k]))

                # TCP 전송
                if client_conn:
                    try:
                        msg = json.dumps(values).encode()
                        client_conn.sendall(msg)
                    except Exception as e:
                        print("[ERROR] Send failed:", e)

        # GUI 멈춤 방지용 짧은 sleep
        cv2.waitKey(1)

    cap.release()



def server_thread():
    global client_conn
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[SERVER] Listening on {HOST}:{PORT}")
        conn, addr = s.accept()
        print(f"[SERVER] Connected by {addr}")
        client_conn = conn
        receive_loop(conn)

# 실행
threading.Thread(target=server_thread, daemon=True).start()
# ArUco 마커 인식 스레드 시작
threading.Thread(target=aruco_thread, daemon=True).start()
root.mainloop()
