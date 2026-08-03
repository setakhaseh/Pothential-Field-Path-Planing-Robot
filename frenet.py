import cv2
import cv2.aruco as aruco
import numpy as np
import keyboard
import time
import math
import numpy as np
import time
from frenetplanner import FrenetPlanner
import websocket
import json
import threading



# # ================= UDP (ESP32) =================
# ESP32_IP = "172.20.10.9"
# UDP_PORT = 4220
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.settimeout(0.01)
# ================= WebSocket (ESP32) =================


# ========= State memory =========
x_prev, y_prev, phi_prev = None, None, None
t_prev = None

# ========= Low-pass filter =========
V_CUTOFF = 3.0  # Hz
v_filt = 0.0
w_filt = 0.0

t0 = time.time()
# ================= UDP (ESP32) =================
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
    return e_theta

#===================PID SETUP====================
pid_v = PID(Kp=1, Ki=0.1, Kd=0.01, out_min=0, out_max=0.6)  # m/s
#pre{slow:[3,1.2,0.06],[3,0.6,0.1]}
pid_w = PID(Kp=4, Ki=0.0, Kd=0.6, out_min=-2.0, out_max=2.0)  # rad/s
v_cmd_base=0.4
v_cmd, w_cmd, w_cmd_prev = 0.0, 0.0, 0.0
x_target = 0.4
y_target = 1.0



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
H_last = None
stopp=False
planner_initialized = False
phi_r=0
ego_s_estimate ,ego_d_estimate=0,0
need_reset = False
#preKs=8.2(OK)

def stop():
    global stopp
    stopp=True
    send(0.0, 0.0)

# ================= Keyboard Events =================
def handle_keys(e):
    global robot_started, target_lane, planner, stopp, ego_s_estimate, ego_d_estimate, x_ref_center, need_reset, planner_initialized

    if e.name in ["1", "2", "3"]:
        target_lane = int(e.name)
        x_center = lane_centers[target_lane]
        x_ref_center = lane_centers[target_lane]
        print(f"Target Lane: {target_lane}")

        # مسیر جدید بساز
        global_route = [[x_center, y, np.pi/2] for y in np.linspace(-1.0, 5.0, 100)]
        need_reset=True
        robot_started=True


    elif e.name in ["4", "s"]:
        stopp = True
        stop()
        robot_started = False
        print("STOP")
def estimate_current_lane(d):
    if d < -0.22:
        return 1
    elif d > 0.22:
        return 3
    else:
        return 2

                

keyboard.on_press(handle_keys)

# ================= Main Loop =================
cap = cv2.VideoCapture(0)
print("▶ System Ready. Press 1, 2, 3 to start/change lane. '4' to Stop. 'Q' to Quit.")
last_time = time.time()  # قبل از while True

try:
    planner = FrenetPlanner()

    # Global reference path (lane center)
    global_route = []
    for y in np.linspace(0, 50.0, 500):
        global_route.append([x_ref_center, y, np.pi/2])


    planner.start(global_route)
    # planner.reset(s=0.0, d=0.0)
    path = None  # قبل از while True
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
                if need_reset:
                        print(f"attemting reset x={x:.2f} y={y:.2f}")
                        print(f"global r :{global_route[0]} to {global_route[-1]}")
                        try:
                            planner.start(global_route)
                            s0 = float(y)
                            d0 = float(x - x_ref_center)
                            planner.reset(s=s0, d=d0)
                            # یکبار مسیر اولیه بساز
                            path, _ = planner.run_step(
                                ego_state=[x, y, 0, 0, phi_r, [[0,0],[0,0]], 10.0],
                                idx=0,
                                change_lane=0,
                                target_speed=0.35
                            )
                            planner_initialized = True
                            need_reset = False
                            print(f"reset at s={s0:.2f} and d={d0:.2f}")
                            print("✅ Planner initialized and path ready")
                        except Exception as e:
                            print(f"⚠ Planner reset failed: {e}")
                # --- Frenet state estimation (for straight reference path) ---
                ego_s_estimate = max(ego_s_estimate, 0.9 * ego_s_estimate + 0.1 * y)
                alpha = 0.7
                ego_d_estimate = alpha * ego_d_estimate + (1 - alpha) * (x - x_ref_center)
                head_g = pts_ground[1][0]
                current_yaw = np.degrees(np.arctan2(head_g[1]-y, head_g[0]-x))
                phi_r = wrap_to_pi(np.radians(current_yaw))
                current_lane = estimate_current_lane(ego_d_estimate)
                change_lane = target_lane - current_lane
                now_t = time.time()
                dt = max(now_t - last_time, 0.001)
                last_time = now_t

                if x_prev is not None:
                    v_filt = math.hypot(x - x_prev, y - y_prev) / dt
                else:
                    v_filt = 0.0
                x_prev, y_prev = x, y
                ego_state = [
                                x,              # x position
                                y,              # y position
                                v_filt,         # speed
                                0.0,            # acceleration (set 0 if unknown)
                                phi_r,          # yaw
                                [[0, 0], [0, 0]],  # velocity & acceleration vectors (dummy)
                                10.0            # track length (any large value)
                            ]

                # if robot_started:
                #     pass
                # # مدیریت دنبال کردن مسیر
                # x_target = x 
                if path is not None and len(path.x)>0:
                        LOOKAHEAD_DIST = 0.15  # meter
                        dist = 0.0
                        target_idx = 0
                        for i in range(1, len(path.x)):
                            dist += math.hypot(
                                path.x[i] - path.x[i-1],
                                path.y[i] - path.y[i-1]
                            )
                            if dist >= LOOKAHEAD_DIST:
                                target_idx = i
                                break

                if robot_started and planner_initialized:
                    if planner.path is None:
                        path, _ = planner.run_step(
                            ego_state=ego_state,
                            idx=0,
                            change_lane=change_lane,
                            target_speed=0.35
                        )
                    if planner.path is not None:
                        path, _ = planner.run_step(
                            ego_state=ego_state,
                            idx=0,
                            change_lane=change_lane,
                            target_speed=0.35
                        )
                    

                        x_target = path.x[target_idx]
                        y_target = path.y[target_idx]


                        e_theta = heading_error(x, y, phi_r, x_target, y_target)
                        error_pos=math.hypot(x_target - x, y_target - y)
                        # v_cmd = pid_v.update(error_pos, dt)
                        v_base = 0.35
                        v_cmd = v_base + pid_v.update(error_pos, dt)
                        w_base=0

                        w_cmd = -pid_w.update(e_theta, dt)
                        if w_cmd<0 :
                            w_base= -0.2

                        if abs(e_theta) > np.deg2rad(5):
                            w_cmd += 0.2 * np.tanh(5 * e_theta)
                        else:
                            w_cmd = 0.7 * w_cmd_prev + 0.3 * w_cmd + w_base
                            w_cmd_prev = w_cmd


                        # w_cmd=0

                        
                        # رسم مسیر روی تصویر (تبدیل معکوس از زمین به پیکسل)
                        H_inv = np.linalg.inv(H_last)
                        # برای رسم مسیر روی تصویر
                        # رسم مسیر روی تصویر (تبدیل معکوس از زمین به پیکسل)
                        H_inv = np.linalg.inv(H_last)
                        points_to_draw = []

                        # برداشتن نقاط مسیر برای رسم (هر 3 نقطه یکبار)
                        for i in range(0, len(path.x), 3):
                            points_to_draw.append([path.x[i], path.y[i]])

                        # تبدیل مختصات مسیر از زمین به تصویر
                        if len(points_to_draw) > 0:
                            pts_to_img = cv2.perspectiveTransform(
                                np.array([points_to_draw], dtype="float32"),
                                H_inv
                            )[0]

                            # رسم نقاط مسیر روی تصویر
                            for pt in pts_to_img:
                                cv2.circle(frame, (int(pt[0]), int(pt[1])), 3, (0, 0, 255), -1)
                    else:
                        print("⚠ Planner path is None! Can't run_step yet.")

                # ارسال داده به ESP32
                now = time.time()
                if not robot_started or stopp or not planner_initialized:
                    send(0.0, 0.0)
                elif now - last_send_time > SEND_INTERVAL:
                    send(v_cmd, w_cmd)
                    last_send_time = now

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


    cap.release()
    cv2.destroyAllWindows()
