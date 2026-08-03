# ===================== IMPORTS =====================
import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import keyboard
import time
import math
import json
import threading
import csv
import websocket

# ================= WebSocket =================
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

def ws_thread():
    global ws
    ws = websocket.WebSocketApp(
        WS_URL,
        on_open=on_open,
        on_close=on_close,
        on_error=on_error
    )
    ws.run_forever()

threading.Thread(target=ws_thread, daemon=True).start()

def send(v, w):
    if not ws_connected:
        return
    ws.send(json.dumps({"v": float(v), "w": float(w)}))
#--------------------extra----------------------
def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def heading_error(xr, yr, phi_r, xt, yt):
    global desired_heading
    desired_heading = np.arctan2(yt - yr, xt - xr)
    e_theta = wrap_to_pi(desired_heading - phi_r)
    e_theta=round(e_theta,2)
    return e_theta

# ================= PID =================
class PID:
    def __init__(self, Kp, Ki, Kd, out_min=-float('inf'), out_max=float('inf')):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.i = 0.0
        self.prev = 0.0
        self.out_min = out_min
        self.out_max = out_max

    def reset(self):
        self.i = 0.0
        self.prev = 0.0

    def update(self, e, dt):
        self.i += e * dt
        self.i = np.clip(self.i, -1.0, 1.0)   # anti-windup
        d = (e - self.prev) / dt if dt > 1e-6 else 0.0
        d = np.clip(d, -10.0, 10.0)
        self.prev = e
        u = self.Kp * e + self.Ki * self.i + self.Kd * d
        return np.clip(u, self.out_min, self.out_max)


pid_w = PID(4.0, 0.0, 0.6, -2.0, 2.0)

# ================= Utils =================
def generate_lane_path(x_lane_target, y_start, length=2.0, n=30):
    """
    مسیر مرجع لاین در مختصات زمین
    """
    ys = np.linspace(y_start, y_start + length, n)
    xs = np.ones_like(ys) * x_lane_target
    return np.stack([xs, ys], axis=1)
prev_robot_pos = None
phi = 0.0
# ================= ArUco =================
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
detector = aruco.ArucoDetector(aruco_dict)

ground_pts_real = {11:[0,0], 12:[1.56,0], 13:[1.56,2.87], 10:[0,2.87]}
GROUND_IDS = [10, 11, 12, 13]

ROBOT_ID = 21
OBSTACLE_IDS = [22,24, 23, 25, 26]  # 4 obstacles

lane_centers = {1:0.29, 2:0.73, 3:1.17}
x_ref_center = lane_centers[2]
target_lane = 2
# ================= HYSTERESIS STATES =================
LANE_FOLLOW  = 0
LANE_CHANGE  = 1
LANE_LOCKED  = 2

lane_state = LANE_FOLLOW

ENTER_TOL = 0.05   # وقتی رسیدی به لاین هدف
EXIT_TOL  = 0.12   # اجازه خروج نمی‌ده (hysteresis)
LOCK_TIME=0.4
lock_start_time=None
lane_change_dir=None

# ================= Potential Field Params =================
K_ATT = 2.0
K_REP = 0.8
REP_RADIUS = 0.7
# ================= Front Tracking Params =================
LOOKAHEAD_Y = 0.25     # 25cm جلوتر برای lane follow

# ================= Potential Field Params =================
K_ATT_Y = 0.2          # مؤلفه جلوبرنده Attractive force
# ================= MAIN CONTROL PART =================
def adaptive_K_rep(dist, K0=1.0, rep_radius=REP_RADIUS):
    if dist >= rep_radius:
        return 0
    # مربع فاصله → حساسیت بیشتر وقتی نزدیک است
    return K0 * ((rep_radius - dist)/rep_radius)**2
def compute_control_pf_hysteresis(decision, current_lane, x, y, phi,
                                  x_lane_target, obstacle_pos, dt):
    global lane_state, target_lane, v_cmd
    global lane_change_dir, lock_start_time
    prev_state = lane_state


    # ================== Decision Logic ==================
    if decision == "CHANGE_LEFT" and target_lane > 1:
        target_lane = current_lane - 1

    elif decision == "CHANGE_RIGHT" and target_lane < 3:
        target_lane = current_lane + 1

    elif decision == "STOP":
        return 0.0, lane_state, 0.0

    # ================== FSM ==================
    if lane_state == LANE_FOLLOW:
        if decision in ["CHANGE_LEFT", "CHANGE_RIGHT"]:
            lane_state = LANE_CHANGE

    elif lane_state == LANE_CHANGE:
        if abs(x - x_lane_target) < ENTER_TOL:
            lane_state = LANE_LOCKED
            lock_start_time = time.time()

    elif lane_state == LANE_LOCKED:
        if time.time() - lock_start_time > LOCK_TIME:
            lane_state = LANE_FOLLOW
    # ===== RESET PID IF STATE CHANGED =====
    if lane_state != prev_state:
        pid_w.reset()
    # ================== Controllers ==================
    v_out = v_cmd

    # ---------- FOLLOW / LOCKED ----------
    if lane_state in [LANE_FOLLOW, LANE_LOCKED]:

        x_lane_center = x_lane_target

        # Frenet lateral error
        d = x - x_lane_center

        # Lookahead target (state-dependent)
        LOOKAHEAD = 0.4
        x_target = x - 0.8 * d
        y_target = y + LOOKAHEAD

        # Desired heading
        theta_des = math.atan2(y_target - y, x_target - x)
        e_theta = wrap_to_pi(theta_des - phi)

        # Speed-aware lateral gain
        k_d = 1.5 * v_cmd
        e = e_theta + math.atan2(k_d * d, v_cmd + 0.05)

        # Angular control (same convention as lane change)
        w_cmd = -pid_w.update(e, dt)

    # ---------- LANE_CHANGE (Potential Field) ----------
    else:
        # Attractive force → مرکز لاین + حرکت رو به جلو
        F_att = np.array([
            K_ATT * (x_lane_target - x),
            K_ATT_Y
        ])

        # Repulsive force → مانع
        F_rep = np.zeros(2)
        if obstacle_pos is not None:
            ox, oy = obstacle_pos
            dx = x - ox
            dy = y - oy
            dist = math.hypot(dx, dy)

            if 0.01 < dist < REP_RADIUS:
                k_rep = adaptive_K_rep(dist)
                F_rep = k_rep * (1/dist - 1/REP_RADIUS) * np.array([dx, dy]) / (dist**3)

        # نیروی نهایی
        F = F_att + F_rep

        # تبدیل نیرو به زاویه هدف
        desired_heading = math.atan2(F[1], F[0])
        e_theta = wrap_to_pi(desired_heading - phi)
        w_cmd = -pid_w.update(e_theta, dt)

    return w_cmd, lane_state, v_out
# ================= Potential Field Params =================

LANE_TOL = 0.05

v_cmd = 0.35
robot_started = False

#====================Make Decision===================
# Decision thresholds
OBSTACLE_LANE_THRESHOLD = 0.22  # 22cm from lane center
LOOKAHEAD_DISTANCE = 0.55  # 40cm
EMERGENCY_DISTANCE = 0.14  # 10cm
LANE_CHECK_DISTANCE = 0.7  # 60cm
lane_detection_history = []
def get_obstacle_lane(obs_x):
    """Determine which lane obstacle is in based on ABSOLUTE X position"""
    # Find closest lane center
    min_distance = float('inf')
    best_lane = None
    
    for lane_num, lane_center_x in lane_centers.items():
        distance = abs(obs_x - lane_center_x)  # ABSOLUTE distance
        if distance < min_distance:
            min_distance = distance
            best_lane = lane_num
    
    # Only assign if within threshold
    if min_distance < OBSTACLE_LANE_THRESHOLD:
        return best_lane
    else:
        return None  # Obstacle not clearly in any lane
def get_current_lane(robot_x):
    """Determine which lane robot is in based on ABSOLUTE X position"""
    if robot_x <= 0:
        return 2  # Default
    
    # Find closest lane center
    min_distance = float('inf')
    best_lane = 2
    
    for lane_num, lane_center_x in lane_centers.items():
        distance = abs(robot_x - lane_center_x)  # ABSOLUTE distance
        if distance < min_distance:
            min_distance = distance
            best_lane = lane_num
    
    # Store in history for stability
    lane_detection_history.append((robot_x, best_lane, min_distance))
    if len(lane_detection_history) > 10:
        lane_detection_history.pop(0)
    
    return best_lane
def make_decision(robot_x, robot_y, obstacle_data):
    """
    Main decision function - returns only the command
    Returns: "KEEP", "STOP", "CHANGE_LEFT", "CHANGE_RIGHT", or "NO_ROBOT"
    """
    # If robot not detected
    if robot_y <= 0:
        return "NO_ROBOT"
    
    # Auto-detect current lane
    current_lane = get_current_lane(robot_x)
    
    # Check for obstacles in current lane (ahead only)
    obstacles_in_my_lane = []
    
    for marker_id, (x, y, lane) in obstacle_data.items():
        if lane == current_lane:
            distance_ahead = y - robot_y
            if 0 < distance_ahead < LOOKAHEAD_DISTANCE:
                obstacles_in_my_lane.append((marker_id, distance_ahead))
    
    # No obstacles in my lane
    if not obstacles_in_my_lane:
        return "KEEP"
    
    # Sort by distance (closest first)
    obstacles_in_my_lane.sort(key=lambda x: x[1])
    closest_distance = obstacles_in_my_lane[0][1]
    
    # Emergency stop check
    if closest_distance < EMERGENCY_DISTANCE:
        return "STOP"
    
    # Check alternative lanes
    left_lane_free = True
    right_lane_free = True
    
    if current_lane > 1:
        for marker_id, (x, y, lane) in obstacle_data.items():
            if lane == current_lane - 1:
                distance_ahead = y - robot_y
                if 0 < distance_ahead < LANE_CHECK_DISTANCE:
                    left_lane_free = False
                    break
    
    if current_lane < 3:
        for marker_id, (x, y, lane) in obstacle_data.items():
            if lane == current_lane + 1:
                distance_ahead = y - robot_y
                if 0 < distance_ahead < LANE_CHECK_DISTANCE:
                    right_lane_free = False
                    break
    
    # Make decision
    if left_lane_free and current_lane > 1:
        return "CHANGE_LEFT"
    elif right_lane_free and current_lane < 3:
        return "CHANGE_RIGHT"
    else:
        return "STOP"
def lane():
    pass

# ================= Keyboard =================
def handle_keys(e):
    global target_lane, robot_started
    if e.name in ["1", "2", "3"]:
        target_lane = int(e.name)
        robot_started = True
        print(f"➡ Target lane {target_lane}")
    elif e.name in ["4", "s"]:
        robot_started = False
        send(0, 0)
        print("⛔ STOP")

keyboard.on_press(handle_keys)

# ================= Camera =================
cap = cv2.VideoCapture(0)
H_last = None
last_time = time.time()

print("▶ Potential Field Lane Change Ready")

# ================= Main Loop =================
while True:
    ret, frame = cap.read()
    if not ret: break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    
    robot_x, robot_y = -1.0, -1.0 # ریست کردن موقعیت در هر فریم
    obstacle_data = {}
    homography_quality = 999.0
    if ids is not None:
        ids = ids.flatten()
        aruco.drawDetectedMarkers(frame, corners, ids)
        image_points, world_points = [], []
        
        # استخراج نقاط زمین برای هموگرافی
        for i, m_id in enumerate(ids):
            if m_id in GROUND_IDS:
                center = np.mean(corners[i][0], axis=0)
                image_points.append(center)
                world_points.append(ground_pts_real[m_id])

        if len(image_points) >= 4:
            H_last, _ = cv2.findHomography(np.array(image_points), np.array(world_points))

        if H_last is not None:
                    total_error = 0
                    valid_points = 0
                    for i, marker_id in enumerate(ids):
                        if marker_id in GROUND_IDS:
                            center = np.mean(corners[i][0], axis=0)
                            pts = np.array([[[center[0], center[1]]]], dtype="float32")
                            pts_ground = cv2.perspectiveTransform(pts, H_last)
                            H_inv = np.linalg.inv(H_last)
                            pts_back = cv2.perspectiveTransform(pts_ground, H_inv)
                            error = np.linalg.norm(center - pts_back[0][0])
                            total_error += error
                            valid_points += 1
                    
                    if valid_points > 0:
                        homography_quality = total_error / valid_points
        if H_last is not None:
                for i, m_id in enumerate(ids):
                    center_img = np.mean(corners[i][0], axis=0)
                    pts = np.array([[[center_img[0], center_img[1]]]], dtype="float32")
                    pts_ground = cv2.perspectiveTransform(pts, H_last)
                    x, y = float(pts_ground[0][0][0]), float(pts_ground[0][0][1])
                    x_lane_target = lane_centers[target_lane]
                    if lane_state in [LANE_FOLLOW, LANE_LOCKED]:
                        if H_last is not None and lane_state in [LANE_FOLLOW, LANE_LOCKED]:
                            # مسیر لاین در زمین
                            path_ground = generate_lane_path(
                                x_lane_target=lane_centers[target_lane],
                                y_start=robot_y,
                                length=1.5,
                                n=20
                            )

                            # ground → image
                            path_ground = path_ground.reshape(-1, 1, 2).astype(np.float32)
                            H_inv = np.linalg.inv(H_last)
                            path_img = cv2.perspectiveTransform(path_ground, H_inv)

                            for k in range(len(path_img) - 1):
                                p1 = tuple(path_img[k][0].astype(int))
                                p2 = tuple(path_img[k+1][0].astype(int))
                                cv2.line(frame, p1, p2, (0, 255, 255), 2)
                    
                    if m_id == ROBOT_ID:
                        robot_x, robot_y = x, y

                        # --- heading از marker ---
                        pts_img = corners[i][0]        # (4,2)
                        head_img = (pts_img[0] + pts_img[1]) / 2

                        pts = np.array([
                            [[center_img[0], center_img[1]]],
                            [[head_img[0], head_img[1]]]
                        ], dtype="float32")

                        pts_ground = cv2.perspectiveTransform(pts, H_last)

                        x_center, y_center = pts_ground[0][0]
                        head_g = pts_ground[1][0]

                        phi = math.atan2(head_g[1] - y_center,
                                        head_g[0] - x_center)
                    elif m_id in OBSTACLE_IDS:
                            # Use get_obstacle_lane() function
                        obs_lane = get_obstacle_lane(x)
                        
                        if obs_lane is not None:
                            obstacle_data[m_id] = (x, y, obs_lane)
                            
                            # Draw obstacle
                            cv2.polylines(frame, [corners[i].astype(int)], True, (255, 0, 0), 2)
                            if robot_y > 0:
                                distance = y - robot_y
                                cv2.putText(frame, f"O{m_id} L{obs_lane} ({distance*100:.0f}cm)", 
                                           (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                            else:
                                cv2.putText(frame, f"O{m_id} L{obs_lane}", 
                                           (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
                        
    # منطق کنترل فقط اگر ربات پیدا شد
    if robot_x != -1.0:
        now = time.time()
        dt = max(now - last_time, 0.01)
        last_time = now

        decision = make_decision(robot_x, robot_y, obstacle_data)
        curr_lane = get_current_lane(robot_x)
        
        nearest_obs = None
        min_d = float('inf')
        for _, (ox, oy, _) in obstacle_data.items():
            d = math.hypot(ox-robot_x, oy-robot_y)
            if d < min_d:
                min_d = d
                nearest_obs = (ox, oy)

        # محاسبه فرمان
        w_cmd, lane_state, v_final = compute_control_pf_hysteresis(
            decision, curr_lane, robot_x, robot_y, phi, 
            lane_centers[target_lane], nearest_obs, dt
        )
        if abs(w_cmd) > 1.0: v_final *= 0.7

        if robot_started:
            send(v_final, w_cmd)

        # ---------------- PRINT TO TERMINAL ----------------
        # print(f"Robot Pos: x={robot_x:.2f}, y={robot_y:.2f}, phi={phi:.2f}")
        # print(f"Target Lane: {target_lane}, Current Lane: {curr_lane}, Decision: {decision}")
        # print(f"v_cmd: {v_final:.2f}, w_cmd: {w_cmd:.2f}, Lane State: {['FOLLOW','CHANGE','LOCKED'][lane_state]}")
        # print("-"*50)

        # ---------------- SHOW ON IMAGE ----------------
        state_str = ["FOLLOW", "CHANGE", "LOCKED"][lane_state]
        cv2.putText(frame, f"Mode: {state_str} | Lane: {target_lane} | Dec: {decision}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Pos:({robot_x:.2f},{robot_y:.2f},{phi:.2f}) v:{v_final:.2f} w:{w_cmd:.2f}", 
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        for p in image_points:
            cv2.circle(frame, (int(p[0]), int(p[1])), 3, (0, 0, 255), -1)

    cv2.imshow("Control", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()
