# Autonomous Omni-Wheel Robot Navigation

This repository contains the final project for our autonomous systems course. It features a fully autonomous, omnidirectional mobile robot built on **ROS 2 Humble**. The system is capable of precise lane tracking, dynamic obstacle detection, and autonomous lane switching using a robust state-machine architecture and custom sensor fusion.

---

## 📂 Repository Structure

The workspace is divided into two primary ROS 2 packages:

### 1. `auto_robot`
Handles the physical description and visualization of the robot.
* **URDF & Meshes:** Contains the physical layout and 3D models of the omni-wheel robot.
* **Config Files:** Stores the parameters for the robot's setup.
* **Launch Files:** Dedicated to displaying and testing the URDF in RViz.

### 2. `auto_simulation`
Contains the core logic, control nodes, and simulation environments.
* **`control/`**: Houses the operational nodes, including the primary state-machine logic (`lane_switching.py`).
* **`worlds/`**: Contains the custom simulation environments.
* **`launch/`**: Launch files to spin up the worlds and control nodes simultaneously.

---

## ⚠️ Dependencies & Prerequisites

**CRITICAL:** This project utilizes a custom-built omnidirectional driver. If you are cloning this repository to test the files, you **must** also clone and build the omni-directional driver in your ROS 2 workspace.

* **Omni-Directional Driver:** [AUSRA-Team/Omni-Directional-Driver](https://github.com/AUSRA-Team/Omni-Directional-Driver)
* **ROS 2 Version:** Humble
* **Sensors Simulated:** LiDAR, depth Camera, IMU, Wheel Encoders

---

## 🧠 Core System Architecture

The robot operates across three primary functional layers:
1. **Perception:** Utilizes LiDAR for spatial awareness and obstacle detection, paired with an simulated OAK-D Lite camera for visual lane tracking.
2. **Localization:** Fuses wheel encoders and IMU data to provide a stable, drift-free coordinate estimate.
3. **Planning & Control:** A state-machine-based node (`lane_switching.py`) manages forward driving, lateral lane changes, and precision stopping at a 10-meter goal.

---

## ⚙️ Technical Highlights

### Sensor Fusion Logic
To overcome the inherent inaccuracies of omni-wheel slippage, a **Manual Fusion Node** was developed:
* **Position Data:** Extracted from the `/odom` topic of the omniwheel driver to track linear progress (X, Y coordinates).
* **Orientation Data:** Overwritten using the `/imu` topic. Because IMUs are immune to the mechanical drift of encoders, this ensures the robot’s yaw remains perfectly calibrated, which is critical for executing precise 0.4m lateral lane shifts.

### Vision Pipeline (Line Following)
Camera tracking relies on a sophisticated Contour-Based Algorithm to identify white lane markings:
* **The Vision Window (ROI):** The system isolates a "Tunnel Vision" box (bottom 40%, center 40% of the image) to ignore adjacent lanes.
* **Otsu’s Binarization:** Automatically calculates the optimal threshold to separate white lines from the grey road, dynamically adapting to changing light in the simulation.
* **Morphological Cleanup:** "Opening" and "Closing" operations remove pixel noise and bridge gaps in dashed lines.
* **Error Calculation:** The horizontal distance between the lane marking's center and the robot's center view is normalized from -1.0 to 1.0. If this error exceeds 2 degrees, a proportional gain (`kp_cam = 0.3`) corrects the robot's yaw.

### Obstacle Detection (LiDAR)
The LiDAR provides a forward safety barrier:
* **Scanning Window:** The robot samples a narrow 6-degree arc directly in front of it (center indices +/- 3).
* **Threshold Trigger:** If any object enters this "flashlight" beam within 1.0 meter, a lateral lane switch is immediately triggered.

---

## 🔄 The Autonomous State Machine

The robot's decision-making "brain" transitions through four logical states based on environmental feedback:

| State | Trigger Condition | Action Taken |
| :--- | :--- | :--- |
| **INIT** | Node startup | Initializes the node and transitions to FORWARD upon receiving the first Odometry data. |
| **FORWARD** | Default driving state | Drives toward the 10m goal. Uses Camera for Yaw correction and Odometry for Lane Centering. |
| **STRAFING** | LiDAR detects obstacle < 1.0m | Kills forward (X) motion; slides laterally to the adjacent lane (y=0.0 to y=0.4). |
| **STOP** | Odometry X-axis >= 10.0m | Kills all velocities and successfully completes the mission. |