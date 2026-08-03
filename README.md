<div align="center">

# Hybrid Autonomous Navigation for Differential Drive Robots

### A Hybrid Navigation Framework Combining Frenet Path Planning, Artificial Potential Field, and Finite State Machine

![Python](https://img.shields.io/badge/Python-3.x-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green)
![ESP32](https://img.shields.io/badge/ESP32-orange)
![WebSocket](https://img.shields.io/badge/WebSocket-RealTime-red)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

</div>

---

# Demo

<p align="center">
<a href="Demo/Demo5.mp4">
<img src="Demo/Demo5.gif" width="900">
</a>
</p>

<p align="center">
<b>Click the animation to watch the full-resolution demonstration video.</b>
</p>

---

# Overview

This project presents a **hybrid autonomous navigation framework** for a differential-drive mobile robot operating in a structured indoor environment.

Instead of relying on a single navigation strategy, the proposed framework combines three complementary approaches:

- **Frenet Coordinate Framework** for smooth trajectory tracking.
- **Artificial Potential Field (APF)** for local obstacle avoidance.
- **Finite State Machine (FSM)** for behavior selection and decision making.

The combination of these methods enables the robot to safely navigate while maintaining smooth motion and reacting intelligently to environmental changes.

---

# Key Features

- Hybrid navigation framework
- Frenet-based trajectory tracking
- Artificial Potential Field obstacle avoidance
- Behavior-based State Machine
- Real-time ArUco localization
- Homography-based coordinate transformation
- Differential drive robot control
- PID velocity controller
- WebSocket communication with ESP32
- Live OpenCV visualization

---

# System Architecture

<p align="center">
<img src="Images/system_architecture.png" width="900">
</p>

The navigation framework consists of five main modules:

1. Environment Perception
2. Localization
3. Decision Making
4. Motion Planning
5. Robot Control

---

# Algorithm Pipeline

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
Obstacle Detection
│
▼
Finite State Machine
│
├─────────────┐
│ │
▼ ▼
Frenet Planner APF Planner
│ │
└──────┬──────┘
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

The navigation system consists of three complementary layers.

---

## 1. Environment Perception

The perception module is responsible for

- Detecting ArUco markers
- Estimating robot pose
- Perspective transformation using homography
- Obstacle detection
- Lane extraction
- Heading estimation

---

## 2. Decision Making

A behavior-based finite state machine determines the robot behavior.

The controller dynamically switches between different navigation modes.

Available states include

- KEEP
- STOP
- CHANGE_LEFT
- CHANGE_RIGHT

Each state is selected according to obstacle positions and lane availability.

---

## 3. Motion Planning

The proposed navigation framework combines two planning methods.

### Frenet Planner

The Frenet planner generates smooth trajectories while the road ahead is clear.

It minimizes steering oscillation and provides stable lane tracking.

---

### Artificial Potential Field

When an obstacle is detected, the Artificial Potential Field planner produces repulsive forces to avoid collisions while preserving forward motion.

---

### Hybrid Controller

The finite state machine continuously determines which planner should be active.

This allows the robot to switch seamlessly between

- Smooth path tracking
- Reactive obstacle avoidance

without interrupting robot motion.

---

# Localization

Robot localization is achieved using computer vision.

The localization pipeline consists of

- ArUco marker detection
- Perspective transformation
- Pose estimation
- Heading estimation
- World coordinate conversion

This approach provides real-time robot position estimation without requiring wheel odometry.

---

# Control System

Robot motion is controlled using PID controllers.

The controller computes

- Linear velocity
- Angular velocity

These commands are transmitted to the ESP32 using WebSocket communication.

---

# Software Stack

- Python
- OpenCV
- NumPy
- WebSocket
- ESP32
- ArUco
- PID Control
- Frenet Planning
- Artificial Potential Field

---

# Project Structure

```
Hybrid-Autonomous-Navigation
│
├── README.md
├── LICENSE
├── requirements.txt
├── .gitignore
│
├── Demo
│ ├── Demo5.mp4
│ └── Demo5.gif
│
├── Images
│ ├── system_architecture.png
│ ├── robot_setup.jpg
│ ├── aruco_detection.png
│ ├── localization.png
│ └── trajectory.png
│
├── src
│ └── Frenet6.py
│
└── docs
```

---

# Installation

Clone the repository

```bash
git clone https://github.com/USERNAME/Hybrid-Autonomous-Navigation.git
```

Install dependencies

```bash
pip install -r requirements.txt
```

Run

```bash
python src/Frenet6.py
```

---

# Results

The proposed framework successfully combines

- Smooth trajectory tracking
- Behavior-based navigation
- Real-time obstacle avoidance
- Stable differential-drive control

The hybrid architecture demonstrates robust navigation performance in structured environments.

---

# Future Work

- ROS2 implementation
- Dynamic obstacle prediction
- MPC controller
- LiDAR integration
- Multi-robot cooperation
- Reinforcement Learning behavior selection

---

# Author

**Setayesh Khasehtarash**

M.Sc. Student in Automation and Control Engineering

Interested in

- Robotics
- Autonomous Navigation
- Computer Vision
- Intelligent Control
- Path Planning

---

# License

This project is licensed under the MIT License.
