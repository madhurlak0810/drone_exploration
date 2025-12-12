#!/bin/bash

echo "=== RViz Launcher with Domain Fix ==="

# Kill any existing RViz processes
pkill -f rviz2

# Clear ROS domain to use default (0)
unset ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

echo "Using ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-default(0)}"

# Navigate to workspace
cd /home/maddy/drone_ws

# Source workspace
source install/setup.bash

echo "Available topics:"
timeout 3 ros2 topic list | grep -E "(map|tf|scan|cmd_vel|odom|robot_description)" || echo "No relevant topics found - check if main system is running"

echo ""
echo "Starting RViz with robust configuration..."
echo "If you see errors, try the troubleshoot_rviz.sh script"
echo ""

# Start RViz with the robust configuration
rviz2 -d src/drone_rl/config/drone_exploration_robust.rviz