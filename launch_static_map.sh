#!/bin/bash

echo "=== Simple Static Map RViz ==="
echo "Launching RViz with minimal configuration..."

# Kill existing RViz
pkill -f rviz2
sleep 2

# Use default domain
unset ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

cd /home/maddy/drone_ws
source install/setup.bash

echo ""
echo "Configuration changes:"
echo "  ✓ Disabled LaserScan (no more graphics errors)"
echo "  ✓ Static map only (no real-time updates)"
echo "  ✓ Reduced frame rate to 10fps"
echo "  ✓ Minimal queue sizes"
echo "  ✓ No map_updates topic"
echo ""
echo "You will see:"
echo "  - Static occupancy grid map"
echo "  - Robot model position"
echo "  - TF transforms"
echo "  - No laser scan visualization"
echo ""

# Start RViz with static configuration
rviz2 -d src/drone_rl/config/drone_exploration_robust.rviz