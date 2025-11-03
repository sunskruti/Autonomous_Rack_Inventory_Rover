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
