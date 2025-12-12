#!/bin/bash
# Launch RViz separately for SLAM map visualization

echo "Launching RViz for SLAM map visualization..."
echo "Make sure the main system is already running!"
echo ""

# Source the workspace
cd ~/drone_ws
source install/setup.bash

# Check if map topic is available
echo "Checking if /map topic is available..."
if ros2 topic list | grep -q "/map"; then
    echo "Map topic found! Launching RViz..."
    echo ""
    
    # Launch RViz with proper config
    rviz2 -d src/drone_rl/config/drone_exploration.rviz
    
else
    echo "Map topic not found. Please make sure the main system is running first."
    echo "Run: ./launch_final.sh"
fi