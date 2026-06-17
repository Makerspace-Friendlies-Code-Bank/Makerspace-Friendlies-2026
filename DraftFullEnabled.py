import socket
import heapq
import time
import tkinter as tk
import threading
import cv2
import requests
import numpy as np
from ultralytics import YOLO
import torch


HOST = '192.168.x.x'               # ESP32's IP address
PORT = 8888
ESP32_CAM_URL = "http://192.168.x.x:81"  # ESP32-CAM stream URL
Detection_sent = False
start_node = 0

#Yolo Model 
model = YOLO("ZYOLO.pt")
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)
print(f"[INFO] Using device: {device}")

def get_next_frame(stream, bytes_data):
    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk
        a = bytes_data.find(b'\xff\xd8')
        b = bytes_data.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            return img, bytes_data
    return None, bytes_data

#ESP32 cam
def stream_video(url):
    global Detection_sent
    try:
        stream = requests.get(url, stream=True)
        bytes_data = b""

        while True:
            img, bytes_data = get_next_frame(stream, bytes_data)
            if img is None:
                continue

            results = model(img)
            annotated = results[0].plot()
            cv2.imshow("ESP32-CAM + YOLO Detection", annotated) #Image with YOLO annotations

             #Detect Bola Kuning
            if results[0].boxes.shape[0] > 0:
                boxes = results[0].boxes.xyxy.cpu().numpy() #move tensore from gpu to cpu and convert to numpy array
                heap = []
                            
                for box in boxes:
                    x1, y1, x2, y2 = box
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2
                    width = x2 - x1
                    height = y2 - y1
                    area = width * height
                    distance_est = 1 / (area + 1e-6)  # smaller box -> further & purpose of +1e-6 to avoid division by zero(means no bola)
                    heapq.heappush(heap, (distance_est, x_center))    
                
                if heap:
                    closest_distance, closest_x_centre = heapq.heappop(heap)
                    frame_width = img.shape[1]
                    x_norm = closest_x_centre / frame_width

                    if not Detection_sent:

                        if x_norm < 0.45: #cant be 0.5 else robot would wobble due to a high sensitivity
                             cmd = "turn_left"

                        elif x_norm > 0.55:
                            cmd = "turn_right"
                                        
                        else:
                            cmd = f"pickup: {closest_distance:.5f}"

                        print("[INFO] Balls detected, sending command to ESP32...")
                        send_simple_command(cmd)

                        if cmd.startswith("pickup"):
                            Detection_sent = True

                        else:
                            Detection_sent = False #keep turning until the ball is centred

            else:
                print("[INFO] No balls detected. Starting rotation...")
                
                while True:
                    degrees = encoder_degrees()
                    if (degrees is not None and degrees >= 360):
                        print("[INFO] Completed full turn. No balls found.")
                        send_simple_command("stop")
                        Detection_sent = False
                        break

                    send_simple_command("turn_right")
                    img, bytes_data = get_next_frame(stream, bytes_data)
                    if img is None:
                        continue

                    results = model(img)
                    if results[0].boxes.shape[0] > 0:
                        print("[INFO] Balls detected during rotation, stopping rotation...")
                        Detection_sent = False #reset so next loop handles it
                        break

            if cv2.waitKey(1) & 0xFF == ord('x'):
                break

    except Exception as e:
        print(f"[CAMERA ERROR] {e}")
    finally:
        cv2.destroyAllWindows()


# Command functions
def send_simple_command(cmd,expect_response=False):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORT))
        sock.sendall(f"{cmd}\n".encode('utf-8'))
        

        if expect_response:
            response= sock.recv(1024).decode('utf-8').strip()
            sock.close()
            return response
        
        else:
            sock.close()
            return None
        
    except Exception as e:
        print(f"[COMMAND ERROR] {e}")

def receive_graph():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {HOST}:{PORT}...")
    sock.connect((HOST, PORT))
    print("Connected.")

    nodes = {}
    total_nodes = 0
    buffer = ""
    end_node = None

    while True:
        data = sock.recv(1024).decode('utf-8')
        if not data:
            break
        buffer += data
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            line = line.strip()
            if line.startswith("NODES:"):
                total_nodes = int(line.split(":")[1])
            elif line.startswith("NODE:"):
                parts = line.split(":")
                node_id = int(parts[1])
                edge_count = int(parts[2])
                edge_data = parts[3].split(",")
                neighbors = []
                distances = []
                for i in range(0, len(edge_data), 2):
                    neighbors.append(int(edge_data[i]))
                    distances.append(float(edge_data[i+1]))
                nodes[node_id] = list(zip(neighbors, distances))
            elif line.startswith("END_NODE"):
                end_node = int(line.split(":")[1])
            if len(nodes) == total_nodes:
                sock.close()
                return nodes, end_node
    sock.close()
    return nodes, end_node

def dijkstra(graph, start):
    total_distance = {node: float('inf') for node in graph}
    total_distance[start] = 0
    prev = {node: None for node in graph}
    queue = [(0, start)]
    while queue:
        current_dist, current_node = heapq.heappop(queue)
        if current_dist > total_distance[current_node]:
            continue
        for neighbor, cost in graph[current_node]:
            distance = current_dist + cost
            if distance < total_distance[neighbor]:
                total_distance[neighbor] = distance
                prev[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))
    return total_distance, prev

def reconstruct_path(prev, start, end):
    path = []
    node = end
    while node is not None:
        path.append(node)
        if node == start:
            break
        node = prev[node]
    path.reverse()
    return path if path[0] == start else []

def reverse_path(path):
    return path[::-1]

def send_path(path):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORT))
        sock.sendall(f"path:{','.join(map(str, path))}".encode('utf-8'))
        sock.close()
    except Exception as e:
        print(f"[PATH SEND ERROR] {e}")

#GUI
def explorecmd():
    send_simple_command("explore", False)

def solvecmd():
    send_simple_command("solve", False)

def reversecmd():
    send_simple_command("reverse", False)

def gocmd():
    send_simple_command("go", False)

def solution():
    graph, finish = receive_graph()
    print("\nFinal graph:")
    for node, edges in graph.items():
        print(f"Node {node}: {edges}")
    _, prev = dijkstra(graph, start_node)
    return reconstruct_path(prev, start_node, finish)

def handle_solve():
    solvecmd()
    time.sleep(2)
    path = solution()
    time.sleep(1)
    send_path(path)


def encoder_degrees():
    response = send_simple_command("encoder degrees", True)
    if response:
        try:
            degrees = float(response.strip())
            return degrees
        except ValueError:
            print(f"[ENCODER ERROR] Invalid degrees value: {response}")
            return None
    else:
        print("[ENCODER ERROR] No response received")
        return None

# ESP32 thread
def start_camera_thread():
    t = threading.Thread(target=stream_video, args=(ESP32_CAM_URL,))
    t.daemon = True
    t.start()

# Start here
if __name__ == "__main__":
    start_camera_thread()

    # Start GUI
    root = tk.Tk()
    root.title("Robot Controller")

    tk.Button(root, text="🧭 Explore", width=25, command=explorecmd).pack(pady=5)
    tk.Button(root, text="🧠 Solve", width=25, command=handle_solve).pack(pady=5)
    tk.Button(root, text="🔁 Reverse", width=25, command=reversecmd).pack(pady=5)
    tk.Button(root, text="🚀 Go", width=25, command=gocmd).pack(pady=5)

    root.mainloop()
