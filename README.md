# Autonomous_Rack_Inventory_Rover
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
