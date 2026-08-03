import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import keyboard
import time
import math

# ================= UDP (ESP32) =================
ESP32_IP = "172.20.10.9"
UDP_PORT = 4220
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.01)

def send(msg):
    try:
        sock.sendto(msg.encode(), (ESP32_IP, UDP_PORT))
    except Exception as e:
        print(f"UDP Error: {e}")

# ================= ArUco Setup =================
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# ================= Ground Coordinates & Config =================
ground_pts_real = {11:[0,0], 12:[1.56,0], 13:[1.56,2.87], 10:[0,2.87]}
ROBOT_ID = 21
lane_centers = {1:0.29, 2:0.73, 3:1.27}
x_ref_center = lane_centers[2]
target_lane = 2


ARRIVAL_THRESHOLD = 0.1
SEND_INTERVAL = 0.05
last_send_time = 0
robot_started = False
traj_d = None
traj_idx = 0
current_s, current_d, current_yaw = 0, 0, 0
H_last = None

def frenet_lane_change(d0, d1, s0, s_points, k=8.2):
    return d0 + (d1-d0)*(1+np.tanh(k*(s_points-s0)))/2

# ================= Keyboard Events =================
def handle_keys(e):
    global traj_d, traj_idx, robot_started, x_ref_center,target_lane
    if e.name in ["1", "2", "3"]:
        target_lane = int(e.name)
        d1 = lane_centers[target_lane] - x_ref_center
        # تولید مسیر از موقعیت فعلی
        s_traj = np.linspace(current_s, current_s + 2.0, 50)
        traj_d = frenet_lane_change(current_d, d1, current_s + 0.2, s_traj)
        traj_idx = 0
        if not robot_started:
            send("0")
            robot_started = True
        print(f"Target Lane: {target_lane}")
    elif e.name == "4" or e.name == "s":
        send("4")
        robot_started = False
        traj_d = None
        print("STOP")
    else:
        robot_started=False
        send("4")
                

keyboard.on_press(handle_keys)

# ================= Main Loop =================
cap = cv2.VideoCapture(0)
print("▶ System Ready. Press 1, 2, 3 to start/change lane. '4' to Stop. 'Q' to Quit.")

try:
    while True:
        ret, frame = cap.read()
        if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            ids = ids.flatten()
            image_points, world_points = [], []
            
            for i, marker_id in enumerate(ids):
                if marker_id in ground_pts_real:
                    center = np.mean(corners[i][0], axis=0)
                    image_points.append(center)
                    world_points.append(ground_pts_real[marker_id])

            # محاسبه ماتریس تبدیل (Homography)
            if len(image_points) >= 4:
                H_last, _ = cv2.findHomography(np.array(image_points), np.array(world_points))

            if H_last is not None and ROBOT_ID in ids:
                idx = np.where(ids==ROBOT_ID)[0][0]
                robot_corners = corners[idx][0]
                center_img = np.mean(robot_corners, axis=0)
                head_img = np.mean([robot_corners[0], robot_corners[1]], axis=0)

                # تبدیل موقعیت ربات به مختصات زمین
                pts = np.array([[[center_img[0], center_img[1]]], [[head_img[0], head_img[1]]]], dtype="float32")
                pts_ground = cv2.perspectiveTransform(pts, H_last)

                x, y = float(pts_ground[0][0][0]), float(pts_ground[0][0][1])
                head_g = pts_ground[1][0]
                current_yaw = np.degrees(np.arctan2(head_g[1]-y, head_g[0]-x))
                current_s, current_d = y, x - x_ref_center

                # مدیریت دنبال کردن مسیر
                x_target = x 
                if robot_started and traj_d is not None:
                    if traj_idx < len(traj_d):
                        d_target = traj_d[traj_idx]
                        x_target = d_target + x_ref_center
                        
                        # رسم مسیر روی تصویر (تبدیل معکوس از زمین به پیکسل)
                        H_inv = np.linalg.inv(H_last)
                        s_traj_visual = np.linspace(current_s, current_s + 1.5, 20)
                        d_traj_visual = frenet_lane_change(current_d, d_target, current_s+0.2, s_traj_visual)
                        
                        points_to_draw = []
                        for i_v in range(len(s_traj_visual)):
                            gx, gy = d_traj_visual[i_v] + x_ref_center, s_traj_visual[i_v]
                            points_to_draw.append([gx, gy])
                        
                        if len(points_to_draw) > 0:
                            pts_to_img = np.array([points_to_draw], dtype="float32")
                            img_pts_back = cv2.perspectiveTransform(pts_to_img, H_inv)[0]
                            for pt in img_pts_back:
                                cv2.circle(frame, (int(pt[0]), int(pt[1])), 3, (0, 0, 255), -1)

                        # منطق پیشروی در مسیر
                        if abs(x - x_target) < ARRIVAL_THRESHOLD:
                            traj_idx += 1
                    else:
                        send("4")

                # ارسال داده به ESP32
                now = time.time()
                if now - last_send_time > SEND_INTERVAL:
                    send(f"{x:.3f},{x_target:.3f},{current_yaw:.1f}")
                    last_send_time = now

                # نمایش اطلاعات
                aruco.drawDetectedMarkers(frame, corners)
                info = f"Pos:({x:.2f},{y:.2f}) Target_X:{x_target:.2f} Yaw:{current_yaw:.1f}"
                cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Frenet Ground Control", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            send("4")
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    sock.close()
