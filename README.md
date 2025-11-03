# 🤖 Autonomous_Rack_Inventory_Rover

A lightweight Python project simulating core functions of an autonomous warehouse rover.

---

## 📂 Files Overview

### `qr_scanning.py`
Detects and decodes QR codes from camera or video feed.  
- Performs blur and lighting correction.  
- Extracts QR data and saves it with timestamp and image snippet in a JSON file.  

### `rack_detection.py`
Simulates rack alignment using virtual distance sensors.  
- Calculates alignment error (left vs right).  
- Uses proportional control (PID concept) to achieve parallel positioning.  
- Visualizes convergence through a simple graph.  

### `motion_control.py`
Simulates rover movement using differential drive kinematics.  
- Computes position and orientation from wheel speeds.  
- Plots the robot’s curved or straight trajectory using Matplotlib.  

---

## 🧠 Tools Used
- Python  
- OpenCV  
- NumPy  
- Matplotlib  

---

## 🏁 Summary
These scripts together represent the vision, control, and motion logic of an autonomous rack inventory rover — focusing purely on simulation and algorithm design without requiring ROS or hardware.

🦾 Autonomous Robot Navigation using ROS2
This project demonstrates autonomous robot navigation using ROS2, Gazebo, and Nav2.
 The robot can automatically follow a saved path (.pth file) or a series of waypoints in simulation or real environment.

🧰 Prerequisites
Tested On:
Ubuntu 22.04


ROS2 Humble / Iron


Gazebo Fortress / Classic


Python 3.10+
🧭 Run the Simulation
1️⃣ Launch Gazebo Environment
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2️⃣ Launch Navigation Stack (Nav2)
In a new terminal:
source ~/ros2_ws/install/setup.bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/path/to/map.yaml


🗺️ Run the Path Follower Script
The path_follower.py node reads a .pth (saved path) file and sends navigation goals to the robot automatically.
Example .pth format
{
  "poses": [
    {"x": 1.2, "y": 0.5, "w": 1.0},
    {"x": 2.0, "y": 1.0, "w": 1.0},
    {"x": 3.5, "y": 1.5, "w": 1.0}
  ]
}

Run the script:
source ~/ros2_ws/install/setup.bash
ros2 run autonomous_nav path_follower.py

The robot will:
Load waypoints from saved_path.pth


Publish goals to /goal_pose


Move autonomously along the saved path



