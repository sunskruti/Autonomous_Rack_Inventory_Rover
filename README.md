# Autonomous_Rack_Inventory_Rover
# 🧠 Autonomous Warehouse Rover – Track A: Systems & Methodology

This project simulates an **autonomous warehouse rover** that performs three major functions:  
1. **QR Code Scanning & Data Management**  
2. **Rack Detection & Precision Alignment**  
3. **X–Y Drive & Motion Control**

It is designed entirely using **Python, OpenCV, NumPy, and Matplotlib**, without any ROS or Gazebo dependency, to demonstrate the core logic and methodology of autonomous rack inventory systems.

---

## 📘 Overview

The goal of this project is to design and simulate the core intelligence of a warehouse rover that can identify rack QR codes, align precisely with racks, and plan its path efficiently.  
This implementation focuses on algorithms and data handling, keeping the simulation lightweight and reproducible on any system.

---

## 🧩 Features

### 🔹 QR Code Scanning & Data Management
- Detects and decodes 5×5 cm QR codes from video or webcam feed.  
- Performs:
  - Blur detection  
  - Lighting adjustment  
  - Perspective correction  
- Stores all decoded QR data in JSON format with:
  - Timestamp  
  - Rack ID  
  - QR Data  
  - Saved image snippet  

**Script:** `qr_scanning.py`  
**Run:**  
```bash
python qr_scanning.py


Open three terminals:

🖥 Terminal 1 — Gazebo
cd ~/turtlebot3_nav_ws/src
source ~/turtlebot3_nav_ws/install/setup.bash
./launch_gazebo.py

🧭 Terminal 2 — Navigation
cd ~/turtlebot3_nav_ws/src
source ~/turtlebot3_nav_ws/install/setup.bash
./launch_nav2.py

🤖 Terminal 3 — Auto Navigation
cd ~/turtlebot3_nav_ws/src
source ~/turtlebot3_nav_ws/install/setup.bash
./auto_nav.py


The robot will automatically navigate through the waypoints you defined.

🛠 Optional Improvements

Replace the static waypoints with QR-code-detected positions or warehouse coordinates.

Add patrolling loops:

while True:
    navigator.followWaypoints(poses)


Use rclpy.spin_once() to integrate live feedback.
