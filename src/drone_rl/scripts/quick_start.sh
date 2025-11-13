#!/bin/bash
# Quick start script for drone exploration

echo "🚁 Setting up Drone RL Exploration Environment..."

# Set ROS2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
cd ~/drone_ws
echo "📦 Building workspace..."
colcon build --packages-select drone_rl

# Source the built workspace
source install/setup.bash

echo "✅ Build complete!"
echo ""
echo "🚀 To launch the exploration simulation:"
echo "   ros2 launch drone_rl sim_launch.py"
echo ""
echo "📊 Available parameters:"
echo "   - headless:=true     (run without GUI)"
echo "   - rviz:=false        (disable RViz)"
echo "   - world:=path/to/world.world"
echo ""
echo "📈 Monitor metrics:"
echo "   ros2 topic echo /exploration_metrics"
echo ""
echo "🎮 Manual control (if needed):"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"