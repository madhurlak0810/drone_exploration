#!/bin/bash
# Quick headless launch for testing
echo "🚁 Starting headless autonomous system..."

cd ~/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start headless Gazebo with ROS factory
echo "Starting Gazebo server..."
gzserver install/drone_rl/share/drone_rl/worlds/exploration_world.world --verbose &
GAZEBO_PID=$!
sleep 5

# Spawn drone
echo "Spawning drone..."
ros2 run gazebo_ros spawn_entity.py \
    -entity drone \
    -file install/drone_rl/share/drone_rl/urdf/drone_simple.urdf \
    -x 0.0 -y 0.0 -z 0.1 &

sleep 3

# Start robot state publisher
echo "Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat install/drone_rl/share/drone_rl/urdf/drone_simple.urdf)" &

sleep 2

# Start SLAM
echo "Starting SLAM..."
ros2 launch slam_toolbox online_async_launch.py &

sleep 3

# Start Q-learning agent
echo "Starting Q-learning agent..."
ros2 run drone_rl q_learning_agent &

sleep 2

# Start metrics
echo "Starting metrics..."
ros2 run drone_rl exploration_metrics &

echo ""
echo "✅ Headless system running!"
echo "Topics to monitor:"
echo "  ros2 topic echo /cmd_vel"
echo "  ros2 topic echo /exploration_metrics"
echo "  ros2 topic list"
echo ""
echo "Press Ctrl+C to stop all processes"

# Wait for user to stop
wait