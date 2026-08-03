Hybrid Autonomous Navigation for Differential Drive Robots

> A hybrid behavior-based navigation framework that combines **Frenet Path Planning**, **Artificial Potential Field (APF)**, and a **Finite State Machine (FSM)** for real-time autonomous navigation of a differential-drive mobile robot.

![Python](https://img.shields.io/badge/Python-3.x-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-green)
![ESP32](https://img.shields.io/badge/ESP32-Robot-orange)
![License](https://img.shields.io/badge/License-MIT-red)

---

# Demo

> **Demo video**

```
Demo/
└── Demo5.mp4
```

*(You can replace this section with a GIF generated from Demo5 for a better GitHub presentation.)*

---

# Overview

This project presents a hybrid autonomous navigation framework designed for a differential-drive mobile robot operating in a structured indoor environment.

Unlike conventional navigation systems that rely on a single path planning strategy, this project combines three complementary approaches:

- **Frenet Coordinate Framework** for trajectory tracking.
- **Artificial Potential Field (APF)** for local obstacle avoidance.
- **Finite State Machine (FSM)** for high-level behavior selection.

The proposed architecture enables the robot to navigate efficiently while adapting its behavior according to the surrounding environment.

---

# Features

- Real-time robot localization using ArUco markers
- Perspective transformation using homography
- Frenet-based trajectory tracking
- Artificial Potential Field obstacle avoidance
- Behavior-based decision making
- State Machine controller
- PID motion controller
- WebSocket communication with ESP32
- Differential drive robot control
- Live visualization using OpenCV

---

# System Architecture

```
Camera
      │
      ▼
ArUco Detection
      │
      ▼
Homography Transformation
      │
      ▼
Robot Localization
      │
      ▼
Environment Perception
      │
      ▼
Finite State Machine
      │
      ├──────────────┐
      │              │
      ▼              ▼
Frenet Planner     APF Planner
      │              │
      └──────┬───────┘
             ▼
      PID Controller
             ▼
      WebSocket Client
             ▼
          ESP32
             ▼
 Differential Drive Robot
```

---

# Navigation Strategy

The navigation framework consists of three major layers.

## 1. Perception

The perception module is responsible for:

- Detecting ArUco markers
- Estimating robot pose
- Transforming image coordinates into world coordinates
- Detecting obstacles
- Extracting lane boundaries

---

## 2. Decision Making

A finite state machine determines the robot behavior.

Available states include:

- KEEP
- STOP
- CHANGE_LEFT
- CHANGE_RIGHT

The state machine continuously evaluates the environment and switches between behaviors according to obstacle positions and lane availability.

---

## 3. Motion Planning

The motion planner combines two complementary methods.

### Frenet Planner

The Frenet planner is responsible for smooth trajectory tracking while the path ahead is clear.

### Artificial Potential Field

Whenever an obstacle is detected, the planner generates a repulsive force to safely avoid collisions.

The controller switches dynamically between these strategies according to the current state.

---

# Localization

Robot localization is achieved using computer vision.

The localization pipeline consists of:

1. Camera image acquisition
2. ArUco marker detection
3. Homography transformation
4. Pose estimation
5. Heading calculation

This provides real-time robot position in the workspace coordinate system.

---

# Control

Robot motion is controlled using PID controllers.

The controller computes:

- Linear velocity
- Angular velocity

These commands are transmitted to the ESP32 via WebSocket communication.

---

# Communication

The software communicates with the robot using:

- WebSocket
- JSON messages
- ESP32 microcontroller

This enables low-latency real-time control.

---

# Technologies

- Python
- OpenCV
- NumPy
- ESP32
- WebSocket
- ArUco
- PID Control
- Frenet Planning
- Artificial Potential Field

---

# Project Structure

```
Hybrid-Autonomous-Navigation/

│
├── README.md
├── LICENSE
├── requirements.txt
│
├── Demo/
│      Demo5.mp4
│
├── Images/
│
├── src/
│      Frenet6.py
│
└── docs/
```

---

# Future Improvements

- Dynamic obstacle prediction
- Multi-robot cooperation
- ROS2 implementation
- LiDAR integration
- Model Predictive Control (MPC)
- Reinforcement Learning based behavior selection

---

# Citation

If you use this project in your research, please cite it appropriately.

---

# License

This project is released under the MIT License.
