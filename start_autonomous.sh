#!/bin/bash
# Autonomous Drone Exploration System Launcher
# Assumes Gazebo world and drone are already running

echo "🚁 STARTING AUTONOMOUS EXPLORATION SYSTEM"
echo "=========================================="

cd ~/drone_ws
source install/setup.bash

echo "🔍 Checking system status..."

# Check if drone is in simulation
if ros2 topic echo /gazebo/model_states --once | grep -q "drone"; then
    echo "✅ Drone detected in Gazebo"
else
    echo "❌ Drone not found in Gazebo"
    exit 1
fi

# Check if LiDAR is working
if ros2 topic list | grep -q "/scan"; then
    echo "✅ LiDAR sensor active"
else
    echo "❌ LiDAR sensor not found"
fi

echo ""
echo "🚀 Starting autonomous components..."

# Start robot state publisher
echo "📡 Starting Robot State Publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(cat src/drone_rl/urdf/drone_simple.urdf)" \
    -p use_sim_time:=true &
RSP_PID=$!
sleep 2

# Start SLAM Toolbox
echo "🗺️ Starting SLAM Toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args \
    -p use_sim_time:=true \
    -p base_frame:=base_link \
    -p odom_frame:=odom \
    -p map_frame:=map \
    -p scan_topic:=/scan \
    -p resolution:=0.05 \
    -p max_laser_range:=10.0 \
    -p minimum_travel_distance:=0.1 \
    -p minimum_travel_heading:=0.1 &
SLAM_PID=$!
sleep 3

# Start Q-Learning Agent
echo "🧠 Starting Q-Learning Agent..."
ros2 run drone_rl q_learning_agent &
QL_PID=$!
sleep 2

# Start Metrics Tracking
echo "📊 Starting Performance Metrics..."
ros2 run drone_rl exploration_metrics &
METRICS_PID=$!
sleep 2

echo ""
echo "🎯 AUTONOMOUS SYSTEM ACTIVE! ✅"
echo "==============================="
echo ""
echo "📈 System Status:"
echo "   🤖 Robot State Publisher: PID $RSP_PID"
echo "   🗺️  SLAM Mapping: PID $SLAM_PID" 
echo "   🧠 Q-Learning Agent: PID $QL_PID"
echo "   📊 Metrics Tracker: PID $METRICS_PID"
echo ""

echo "📡 Active ROS Topics:"
sleep 2
ros2 topic list | grep -E "(scan|odom|map|cmd_vel|metrics)" | sort | while read topic; do
    echo "   ✓ $topic"
done
echo ""

echo "🎮 Monitoring Commands:"
echo "   • View map: rviz2 (add Map display on /map topic)"
echo "   • Monitor metrics: ros2 topic echo /exploration_metrics"
echo "   • Check drone movement: ros2 topic echo /cmd_vel"
echo "   • View LiDAR: ros2 topic echo /scan --once"
echo ""

echo "🔍 Live Performance:"
echo "   • Coverage area expanding as drone explores"
echo "   • Q-table learning optimal exploration paths"
echo "   • SLAM building real-time occupancy grid"
echo "   • Obstacle avoidance using LiDAR feedback"
echo ""

echo "⏹️  Press Ctrl+C to shutdown autonomous system"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down autonomous system..."
    kill $METRICS_PID $QL_PID $SLAM_PID $RSP_PID 2>/dev/null
    echo "✅ All autonomous components stopped"
    echo "💾 Q-table and metrics saved"
    exit 0
}

# Set trap for clean shutdown
trap cleanup INT TERM

echo "🚀 AUTONOMOUS EXPLORATION RUNNING..."
echo "   Drone is learning to explore and map autonomously!"

# Keep script running and monitor
while kill -0 $QL_PID 2>/dev/null; do
    sleep 5
    echo -n "⚡"  # Activity indicator
done

echo ""
echo "⚠️  Q-learning agent stopped. Shutting down system..."
cleanup