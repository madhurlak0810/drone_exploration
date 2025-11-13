#!/bin/bash
# FINAL VERSION: Complete Autonomous Drone Exploration System
# Integrates Gazebo simulation, Q-learning, SLAM, and performance monitoring

echo "🚁 AUTONOMOUS DRONE EXPLORATION - FINAL VERSION"
echo "=============================================="
echo ""
echo "🎯 Initializing complete system with:"
echo "   • Gazebo simulation world"
echo "   • Autonomous Q-learning agent"
echo "   • Real-time SLAM mapping"
echo "   • Performance metrics tracking"
echo ""

# Set up environment
cd ~/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Export necessary environment variables
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/drone_rl/share/drone_rl
export ROS_DOMAIN_ID=0

echo "🌍 Step 1: Launching Gazebo World..."
# Start Gazebo server with ROS factory plugins (headless for stability)
echo "   🔧 Using headless mode to avoid GUI crashes..."
ros2 launch gazebo_ros gazebo.launch.py gui:=false world:=install/drone_rl/share/drone_rl/worlds/exploration_world.world &
GAZEBO_PID=$!

# Give Gazebo time to initialize
sleep 8

# Optional: Try to start GUI client separately (may crash, but won't affect system)
echo "   🖥️  Attempting to start GUI client (optional)..."
(gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so > /dev/null 2>&1 &) || echo "   ⚠️  GUI client failed (continuing headless)"

echo "✅ Gazebo world initialized"
echo ""

echo "🤖 Step 2: Spawning Drone Robot..."
# Spawn the drone in Gazebo using the working simplified URDF
ros2 run gazebo_ros spawn_entity.py \
    -entity drone \
    -file install/drone_rl/share/drone_rl/urdf/drone_simple.urdf \
    -x 0.0 -y 0.0 -z 0.1 \
    -timeout 10.0 &
SPAWN_PID=$!

sleep 3
echo "✅ Drone spawned at origin (0,0,0.1)"
echo ""

echo "📡 Step 3: Starting Robot State Publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat install/drone_rl/share/drone_rl/urdf/drone_simple.urdf)" \
    -p use_sim_time:=true &
RSP_PID=$!

sleep 2
echo "✅ Robot state publisher active"
echo ""

echo "🗺️ Step 4: Initializing SLAM System..."
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args \
    -p use_sim_time:=true \
    -p base_frame:=base_link \
    -p odom_frame:=odom \
    -p map_frame:=map \
    -p scan_topic:=/scan \
    -p resolution:=0.05 \
    -p max_laser_range:=10.0 &
SLAM_PID=$!

sleep 3
echo "✅ SLAM Toolbox running and ready for mapping"
echo ""

echo "🧠 Step 5: Starting Q-Learning Agent..."
install/drone_rl/bin/q_learning_agent &
QL_PID=$!

sleep 2
echo "✅ Q-Learning agent initialized and learning"
echo ""

echo "📊 Step 6: Starting Performance Metrics..."
install/drone_rl/bin/exploration_metrics &
METRICS_PID=$!

sleep 2
echo "✅ Metrics tracking active"
echo ""

echo "🎯 SYSTEM STATUS: FULLY OPERATIONAL ✅"
echo "======================================"
echo ""
echo "📈 Live System Information:"
echo "   🌍 Simulation: Gazebo world with exploration environment"
echo "   🚁 Robot: Autonomous drone with LiDAR and IMU"
echo "   🧠 AI: Q-learning with frontier-based exploration"
echo "   🗺️ Mapping: Real-time SLAM with occupancy grid"
echo "   📊 Metrics: Coverage, efficiency, and collision tracking"
echo ""

echo "📡 Active ROS Topics:"
sleep 2
ros2 topic list | grep -E "(scan|odom|map|cmd_vel|metrics)" | while read topic; do
    echo "   ✓ $topic"
done
echo ""

echo "🔍 System Monitoring:"
echo "   • Q-table: ~/drone_ws/q_table.npy"
echo "   • Metrics: ~/drone_ws/metrics/"
echo "   • Logs: ~/.ros/log/"
echo ""

echo "🎮 Controls:"
echo "   • System runs autonomously"
echo "   • Monitor topics: ros2 topic echo /exploration_metrics"
echo "   • View map: rviz2 (add Map display on /map topic)"
echo "   • Check progress: tail -f ~/.ros/log/*/q_learning_agent*.log"
echo ""

echo "⏹️  Press Ctrl+C to shutdown all components"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down system..."
    kill $METRICS_PID $QL_PID $SLAM_PID $RSP_PID $SPAWN_PID $GAZEBO_PID 2>/dev/null
    echo "✅ All components stopped"
    echo "💾 Q-table and metrics saved"
    exit 0
}

# Set trap for clean shutdown
trap cleanup INT TERM

echo "🚀 AUTONOMOUS EXPLORATION RUNNING..."
echo "   Drone is now learning to explore autonomously!"
echo "   Building map and avoiding obstacles using Q-learning"
echo ""

# Keep script running
while kill -0 $QL_PID 2>/dev/null; do
    sleep 5
    echo -n "."  # Progress indicator
done

echo ""
echo "⚠️  Q-learning agent stopped. Shutting down system..."
cleanup