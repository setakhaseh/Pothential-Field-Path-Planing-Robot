import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import keyboard
import time
import math
import numpy as np
import time



# # ================= UDP (ESP32) =================
# ESP32_IP = "172.20.10.9"
# UDP_PORT = 4220
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.settimeout(0.01)
# ================= WebSocket (ESP32) =================
import csv

# ========= Logging =========
LOG_FILE = "vw_log.csv"
log_file = open(LOG_FILE, "w", newline="")
log_writer = csv.writer(log_file)

# header مخصوص MATLAB
log_writer.writerow([
    "t",
    "x",
    "y",
    "theta",
    "v_cmd",
    "w_cmd"
])

# ========= State memory =========
x_prev, y_prev, phi_prev = None, None, None
t_prev = None

# ========= Low-pass filter =========
V_CUTOFF = 3.0  # Hz
v_filt = 0.0
w_filt = 0.0

t0 = time.time()

import websocket
import json
import threading

ESP32_IP = "172.20.10.9"
WS_URL = f"ws://{ESP32_IP}:80/"

ws = None
ws_connected = False

def on_open(ws_):
    global ws_connected
    ws_connected = True
    print("✅ WebSocket connected")

def on_close(ws_, a, b):
    global ws_connected
    ws_connected = False
    print("❌ WebSocket closed")

def on_error(ws_, error):
    print("WS Error:", error)

def on_message(ws_, message):
    # این همون لاگیه که ESP32 می‌فرسته (PWM + DIR)
    print("ESP32:", message)

def ws_thread():
    global ws
    ws = websocket.WebSocketApp(
        WS_URL,
        on_open=on_open,
        on_close=on_close,
        on_error=on_error,
        on_message=on_message
    )
    ws.run_forever()

threading.Thread(target=ws_thread, daemon=True).start()

# ----------------- PID کلاس -----------------
class PID:
    def __init__(self, Kp, Ki, Kd, out_min=-float('inf'), out_max=float('inf')):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0
        self.out_min = out_min
        self.out_max = out_max

    def update(self, error, dt):

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        out = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return max(min(out, self.out_max), self.out_min)

desired_heading=0

# def send(msg):
#     try:
#         sock.sendto(msg.encode(), (ESP32_IP, UDP_PORT))
#     except Exception as e:
#         print(f"UDP Error: {e}")
def send(v, w):
    if not ws_connected:
        return
    msg = json.dumps({
        "v": float(v),
        "w": float(w)
    })
    ws.send(msg)

#--------------------extra----------------------
def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def heading_error(xr, yr, phi_r, xt, yt):
    global desired_heading
    desired_heading = np.arctan2(yt - yr, xt - xr)
    e_theta = wrap_to_pi(desired_heading - phi_r)
    e_theta=round(e_theta,2)
    return e_theta

#===================PID SETUP====================
pid_v = PID(Kp=1, Ki=0.1, Kd=0.01, out_min=0, out_max=0.6)  # m/s
#pre{slow:[3,1.2,0.06],[3,0.6,0.1]}
pid_w = PID(Kp=4, Ki=0.0, Kd=0.6, out_min=-2.0, out_max=2.0)  # rad/s
v_cmd_base=0.4
v_cmd, w_cmd, w_cmd_prev = 0.0, 0.0, 0.0



# ================= ArUco Setup =================
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# ================= Ground Coordinates & Config =================
ground_pts_real = {11:[0,0], 12:[1.56,0], 13:[1.56,2.87], 10:[0,2.87]}
ROBOT_ID = 21
lane_centers = {1:0.29, 2:0.73, 3:1.17}
x_ref_center = lane_centers[2]
target_lane = 2


ARRIVAL_THRESHOLD = 0.1
#pre{0.05}
SEND_INTERVAL = 0.02
last_send_time = 0
robot_started = False
traj_d = None
current_s, current_d, current_yaw = 0, 0, 0
H_last = None
stopp=None
phi_r=0
#preKs=8.2(OK)
def frenet_lane_change(d0, d1, s0, s_points, k=8.2):
    return d0 + (d1-d0)*(1+np.tanh(k*(s_points-s0)))/2
def stop():
    send(0.0, 0.0)

# ================= Keyboard Events =================
def handle_keys(e):
    global traj_d, traj_idx, robot_started, x_ref_center,target_lane,stopp
    if e.name in ["1", "2", "3"]:
        target_lane = int(e.name)
        d1 = lane_centers[target_lane] - x_ref_center
        # تولید مسیر از موقعیت فعلی
        # s_traj = np.linspace(current_s, current_s + 2.0, 50)
        # traj_d = frenet_lane_change(current_d, d1, current_s + 0.2, s_traj)
        # traj_idx = 0
        # if not robot_started:
        #     send("0")
        robot_started = True
        print(f"Target Lane: {target_lane}")
    elif e.name == "4" or e.name == "s":
        stopp=True
        stop()
        robot_started = False
        print("STOP")

                

keyboard.on_press(handle_keys)

# ================= Main Loop =================
cap = cv2.VideoCapture(0)
print("▶ System Ready. Press 1, 2, 3 to start/change lane. '4' to Stop. 'Q' to Quit.")
last_time = time.time()  # قبل از while True

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
                phi_r = np.radians(current_yaw)

                current_s, current_d = y, x - x_ref_center
                x_target = x
                y_target=y
                t_now = time.time()
                t_rel = t_now - t0

                if t_prev is not None:
                            dt = t_now - t_prev

                            if dt > 1e-4:
                                # ===== Measured velocities =====
                                dx = x - x_prev
                                dy = y - y_prev
                                dphi = (phi_r - phi_prev + math.pi) % (2*math.pi) - math.pi


                                v_meas = math.sqrt(dx*dx + dy*dy) / dt
                                w_meas = dphi / dt

                                # ===== Low-pass filter =====
                                alpha = math.exp(-2 * math.pi * V_CUTOFF * dt)
                                v_filt = alpha * v_filt + (1 - alpha) * v_meas
                                w_filt = alpha * w_filt + (1 - alpha) * w_meas

                                # ===== Log =====
                                if robot_started:
                                    log_writer.writerow([
                                    t_rel,
                                    x,
                                    y,
                                    phi_r,
                                    v_cmd,
                                    w_cmd          
                                ])
                                

                        # ===== Update memory =====
                x_prev, y_prev, phi_prev = x, y, phi_r        
                t_prev = t_now

                if robot_started:
                    d1 = lane_centers[target_lane] - x_ref_center
                    S_FORWARD = 0.08  # مسیر کمی جلوتر از ربات شروع شود
                    s_traj = np.linspace(current_s + S_FORWARD, current_s + 2 + S_FORWARD, 50)
                    traj_d = frenet_lane_change(current_d, d1, current_s + 0.2 + S_FORWARD, s_traj)


                # # مدیریت دنبال کردن مسیر
                # x_target = x 
                if robot_started and traj_d is not None:
                        

                        #look aheads=3cm(slow),5(slow),8(better)
                        LOOKAHEAD_IDX = 0.2
                        # پیدا کردن نزدیک‌ترین نقطه جلوتر از current_s
                        idx_la = np.searchsorted(s_traj, current_s + LOOKAHEAD_IDX)  # 5cm جلوتر
                        idx_la = min(idx_la, len(traj_d)-1)
                        d_target = traj_d[idx_la]
                        x_target = d_target + x_ref_center
                        y_target = s_traj[idx_la]


                        e_theta = heading_error(x, y, phi_r, x_target, y_target)
                        dt = max(time.time() - last_time, 0.001)
                        last_time = time.time()
                        error_pos=math.hypot(x_target - x, y_target - y)
                        # v_cmd = pid_v.update(error_pos, dt)
                        v_base = 0.35
                        v_cmd = v_base + pid_v.update(error_pos, dt)
                        w_base=0

                        w_cmd = -pid_w.update(e_theta, dt)
                        if w_cmd<0 :
                            w_base= -0.2

                        if abs(e_theta) > np.deg2rad(5):
                            w_cmd = w_cmd + math.copysign(0.2,w_cmd )
                        else:
                            w_cmd = 0.7 * w_cmd_prev + 0.3 * w_cmd + w_base
                            w_cmd_prev = w_cmd


                        # w_cmd=0

                        
                        # رسم مسیر روی تصویر (تبدیل معکوس از زمین به پیکسل)
                        H_inv = np.linalg.inv(H_last)
                        # برای رسم مسیر روی تصویر
                        S_FORWARD = 0.08  # مسیر کمی جلوتر از ربات شروع شود
                        s_traj_v = np.linspace(current_s + S_FORWARD, current_s + 2 + S_FORWARD, 30)
                        d_traj_v = frenet_lane_change(current_d, d1, current_s + 0.2 + S_FORWARD, s_traj)

                        points_to_draw = []
                        for i_v in range(len(s_traj_v)):
                            gx, gy = d_traj_v[i_v] + x_ref_center, s_traj_v[i_v]
                            points_to_draw.append([gx, gy])
                        
                        if len(points_to_draw) > 0:
                            pts_to_img = np.array([points_to_draw], dtype="float32")
                            img_pts_back = cv2.perspectiveTransform(pts_to_img, H_inv)[0]
                            for pt in img_pts_back:
                                cv2.circle(frame, (int(pt[0]), int(pt[1])), 3, (0, 0, 255), -1)


                # ارسال داده به ESP32
                now = time.time()
                if  stopp:
                    send(0, 0)

                    
                elif now - last_send_time > SEND_INTERVAL:
                        send(v_cmd, w_cmd)
                        last_send_time = time.time()

                # نمایش اطلاعات
                aruco.drawDetectedMarkers(frame, corners)
                # info = f"Pos:({x:.2f},{y:.2f}) Target_X:{x_target:.2f} Target_Y:{y_target:.2f}\n" \
                #     f"v_cmd:{v_cmd:.2f} w_cmd:{w_cmd:.2f}"

                # cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                line1 = f"Pos:({x:.2f},{y:.2f},{phi_r:.2f}) Target_X:{x_target:.2f} Target_Y:{y_target:.2f}"
                line2 = f"v_cmd:{v_cmd:.2f} w_cmd:{w_cmd:.2f} yaw_target:{desired_heading:.2f}"
                cv2.putText(frame, line1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.putText(frame, line2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)


        cv2.imshow("Frenet Ground Control", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop()

            break

finally:
    log_file.close()
    print("Log saved to", LOG_FILE)

    cap.release()
    cv2.destroyAllWindows()
    socket.close()
