#!/bin/bash

# Set the TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# 1. Launch Gazebo with TurtleBot3
gnome-terminal --title="Gazebo" -- bash -c "
    echo 'Starting Gazebo with TurtleBot3...';
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py;
    exec bash"

# Wait for Gazebo to initialize
sleep 5

# 2. Launch Nav2 (requires Gazebo already running)
gnome-terminal --title="Nav2" -- bash -c "
    echo 'Starting Nav2...';
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true;
    exec bash"

# Launch AMCL (Localization)
gnome-terminal --title="AMCL" -- bash -c "
    echo 'Starting AMCL...';
    ros2 launch nav2_bringup localization_launch.py use_sim_time:=true;
    exec bash"
sleep 2  # Wait for AMCL    

# Wait for Nav2 to initialize
sleep 3

# 3. Launch RViz2 with Nav2 plugins
gnome-terminal --title="RViz2" -- bash -c "
    echo 'Starting RViz2...';
    ros2 launch nav2_bringup rviz_launch.py;
    exec bash"

# 4. Launch teleop keyboard (optional)
gnome-terminal --title="Teleop" -- bash -c "
    echo 'Starting Teleop Keyboard...';
    ros2 run turtlebot3_teleop teleop_keyboard;
    exec bash"

echo -e "\n\033[1;32mSimulation is ready!\033[0m"
echo "- Gazebo: Robot and world simulation."
echo "- Nav2: Navigation stack (planner, controller, AMCL)."
echo "- RViz2: Visualization with Nav2 plugins."
echo "- Teleop: Manual control (W/A/S/D)."