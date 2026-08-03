import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import keyboard
import time

# ================= UDP (ESP32) =================
ESP32_IP = "172.20.10.9"
UDP_PORT = 4220
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.1)

def send(msg):
    try:
        sock.sendto(msg.encode(), (ESP32_IP, UDP_PORT))
    except Exception as e:
        print(f"UDP Error: {e}")

# ================= ArUco =================
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# ========== Ground Coordinates (meters) ==========
ground_pts_real = {11:[0,0], 12:[1.56,0], 13:[1.56,2.87], 10:[0,2.87]}
ROBOT_ID = 21
lane_centers = {1:0.29, 2:0.78, 3:1.27}
x_ref_center = lane_centers[2]

# ========== Control Variables ==========
traj_d = None
traj_idx = 0
H_last = None
ARRIVAL_THRESHOLD = 0.08
MAX_ALLOWED_ERROR = 0.4
SEND_INTERVAL = 0.05
last_send_time = 0
robot_started = False

def frenet_lane_change(d0, d1, s0, s_points, k=1.2):
    return d0 + (d1-d0)*(1+np.tanh(k*(s_points-s0)))/2

# ================= Camera =================
cap = cv2.VideoCapture(0)
last_key = None

print("▶ System Ready. 1/2/3:Lane+Start, 4:Stop, Q:Quit")

try:
    while True:
        ret, frame = cap.read()
        if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            ids = ids.flatten()
            aruco.drawDetectedMarkers(frame, corners)

            image_points, world_points = [], []
            for i, marker_id in enumerate(ids):
                if marker_id in ground_pts_real:
                    center = np.mean(corners[i][0], axis=0)
                    image_points.append(center)
                    world_points.append(ground_pts_real[marker_id])

            if len(image_points) >= 4:
                H_last, _ = cv2.findHomography(np.array(image_points), np.array(world_points))

            if H_last is not None and ROBOT_ID in ids:
                idx = np.where(ids==ROBOT_ID)[0][0]
                robot_corners = corners[idx][0]
                center_img = np.mean(robot_corners, axis=0)
                head_img = np.mean([robot_corners[0], robot_corners[1]], axis=0)

                pts = np.array([[[center_img[0], center_img[1]]], [[head_img[0], head_img[1]]]], dtype="float32")
                pts_ground = cv2.perspectiveTransform(pts, H_last)

                x, y = float(pts_ground[0][0][0]), float(pts_ground[0][0][1])
                head_g = pts_ground[1][0]
                yaw = np.degrees(np.arctan2(head_g[1]-y, head_g[0]-x))

                s = y
                d = x - x_ref_center

                # ========== Closed Loop Logic (3-Parameter Send) ==========
                current_time = time.time()
                if robot_started:
                    # تعیین X هدف
                    if traj_d is not None and traj_idx < len(traj_d):
                        d_target = traj_d[traj_idx]
                        x_target = d_target + x_ref_center
                        if np.abs(x - x_target) < ARRIVAL_THRESHOLD:
                            traj_idx += 1
                    else:
                        x_target = x # اگر مسیری نبود، درجای خود بماند

                    # ارسال داده: x_actual, x_target, yaw
                    if current_time - last_send_time > SEND_INTERVAL:
                        send(f"{x:.4f},{x_target:.4f},{yaw:.1f}")
                        last_send_time = current_time
                    d_ref_display = x_target - x_ref_center
                else:
                    d_ref_display = d

                info = f"S:{s:.2f} D:{d:.2f} Target_D:{d_ref_display:.2f} Yaw:{yaw:.1f}"
                cv2.putText(frame, info, (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.imshow("Frenet Control", frame)

        # ========== Keyboard ==========
        key = None
        if keyboard.is_pressed("1"): key="1"
        elif keyboard.is_pressed("2"): key="2"
        elif keyboard.is_pressed("3"): key="3"
        elif keyboard.is_pressed("4") or keyboard.is_pressed("s"): key="4"

        if key and key != last_key:
            if key in ["1","2","3"]:
                if not robot_started:
                    send("0")
                    robot_started = True
                target_lane = int(key)
                d0, d1 = d, lane_centers[target_lane] - x_ref_center
                s0, s_traj = s + 0.1, np.linspace(s, s+1.5, 100)
                traj_d = frenet_lane_change(d0, d1, s0, s_traj)
                traj_idx = 0
            elif key == "4":
                send("4")
                robot_started = False
            last_key = key
        elif not key: last_key = None

        if (cv2.waitKey(1)&0xFF==ord('q')) or keyboard.is_pressed("q"):
            send("4")
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    sock.close()