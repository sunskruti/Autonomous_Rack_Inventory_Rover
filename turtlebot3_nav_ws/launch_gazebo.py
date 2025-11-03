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
