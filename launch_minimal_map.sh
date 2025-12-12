#!/bin/bash

echo "=== MINIMAL MAP DISPLAY (NO YELLOW FLICKER) ==="

# Kill all existing RViz
pkill -f rviz2
sleep 2

# Use default domain
unset ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

cd /home/maddy/drone_ws
source install/setup.bash

echo ""
echo "Using ultra-minimal configuration:"
echo "  ✓ Only Grid, Map, and RobotModel"
echo "  ✓ No LaserScan (eliminates graphics issues)"
echo "  ✓ No TF display (reduces complexity)"
echo "  ✓ No Odometry trail (prevents accumulation)"
echo "  ✓ No Update Topic (prevents flicker)"
echo "  ✓ Frame rate: 5fps (minimal load)"
echo "  ✓ Single durability policy (no conflicts)"
echo ""

# Check if topics exist
echo "Checking for map topic..."
timeout 2 ros2 topic echo /map --once > /dev/null 2>&1 && echo "✓ Map topic available" || echo "⚠ Map topic not found - start main system first"

echo ""
echo "Starting minimal RViz..."
rviz2 -d src/drone_rl/config/drone_minimal.rviz