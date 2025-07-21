import tkinter as tk
from tkinter import ttk
import socket
import threading
import json

HOST = '0.0.0.0'
PORT = 12345

client_conn = None
labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

root = tk.Tk()
root.title("🤖 Robot Control Server GUI")
# root.geometry("420x450")  ← ❌ 창 크기 고정하지 않음!
root.resizable(False, False)

style = ttk.Style()
style.configure("TLabel", font=("Arial", 11))
style.configure("TButton", font=("Arial", 11))
style.configure("Feedback.TLabel", font=("Arial", 11, "bold"), foreground="blue")

# 입력값 구역
input_frame = ttk.LabelFrame(root, text="📝 입력값", padding=10)
input_frame.pack(fill="x", padx=10, pady=5)

entries = {}
for i, label in enumerate(labels):
    ttk.Label(input_frame, text=label).grid(row=i, column=0, sticky="e", pady=2)
    entry = ttk.Entry(input_frame, width=15)
    entry.grid(row=i, column=1, padx=5, pady=2)
    entries[label] = entry

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

def send_go():
    if client_conn:
        try:
            print("[DEBUG] client_conn is valid.")
            data = {k: float(entries[k].get()) for k in labels}
            msg = json.dumps(data).encode()
            client_conn.sendall(msg)
            print("[SERVER] Sent GO:", data)
        except ValueError:
            print("[ERROR] Invalid input format!")
    else:
        print("[ERROR] client_conn is None. Not connected to robot.")

def send_stop():
    if client_conn:
        client_conn.sendall(b's')
        print("[SERVER] Sent STOP signal ('s')")

go_button = ttk.Button(button_frame, text="▶ Go", command=send_go)
go_button.grid(row=0, column=0, padx=20, pady=5)

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
root.mainloop()
