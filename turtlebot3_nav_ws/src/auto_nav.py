#!/usr/bin/env python3
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to be active
    print("⏳ Waiting for Nav2 lifecycle nodes to activate...")
    navigator.waitUntilNav2Active()

    # Define waypoints (x, y, yaw)
    waypoints = [
        (2.0, 0.0, 0.0),
        (2.0, 2.0, 1.57),
        (0.0, 2.0, 3.14),
        (0.0, 0.0, -1.57)
    ]

    poses = []
    for (x, y, yaw) in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        poses.append(pose)

    print("🚗 Sending waypoints...")
    navigator.followWaypoints(poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"🧭 Traveling to waypoint {feedback.current_waypoint + 1}/{len(poses)}")

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("✅ Reached all waypoints successfully!")
    else:
        print("❌ Navigation failed or canceled.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import os
import subprocess
import time

def launch_gazebo():
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    print("🚀 Launching Gazebo world...")
    subprocess.Popen([
        'ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'
    ])
    time.sleep(10)  # wait for Gazebo to load

if __name__ == "__main__":
    launch_gazebo()

#!/usr/bin/env python3
import os
import subprocess
import time

def launch_nav2():
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    print("🧭 Launching Navigation2 stack...")
    subprocess.Popen([
        'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 'use_sim_time:=True'
    ])
    time.sleep(15)  # wait for Nav2 to fully start

if __name__ == "__main__":
    launch_nav2()
