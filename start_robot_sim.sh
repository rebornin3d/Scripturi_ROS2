#!/bin/bash

# Set the TurtleBot3 model (change to 'waffle' if needed)
export TURTLEBOT3_MODEL=burger

# Function to check if a ROS 2 node is running
check_node() {
    if ros2 node list | grep -q "$1"; then
        echo "[SUCCESS] $1 is running."
    else
        echo "[ERROR] $1 failed to start!"
        exit 1
    fi
}

# Launch Gazebo with TurtleBot3 in a new terminal
gnome-terminal --title="Gazebo" -- bash -c "
    echo 'Starting Gazebo with TurtleBot3...';
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py;
    exec bash"

sleep 5  # Wait for Gazebo to initialize

# Launch RViz2 in another terminal
gnome-terminal --title="RViz2" -- bash -c "
    echo 'Starting RViz2...';
    ros2 launch turtlebot3_bringup rviz2.launch.py;
    exec bash"

sleep 3  # Wait for RViz2 to start

# Launch teleop keyboard in a third terminal
gnome-terminal --title="Teleop" -- bash -c "
    echo 'Starting Teleop Keyboard...';
    ros2 run turtlebot3_teleop teleop_keyboard;
    exec bash"

# Verify critical nodes are running
check_node "gazebo"
check_node "rviz2"
check_node "teleop_keyboard"

echo -e "\n\033[1;32mSimulation is ready!\033[0m"
echo "- Gazebo: Visualize the robot and world."
echo "- RViz2: View LIDAR, TF, and sensor data."
echo "- Teleop: Use W/A/S/D to move the robot."