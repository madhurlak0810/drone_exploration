#!/bin/bash

echo "Starting autonomous drone exploration system with proper robot spawning..."

# Source the workspace
source ~/drone_ws/install/setup.bash

# Kill any existing processes
echo "Cleaning up any existing processes..."
pkill -f "gzserver\|gzclient\|gazebo\|robot_state_publisher\|slam_toolbox\|q_learning" || true
sleep 2

# Start Gazebo server (headless)
echo "Starting Gazebo server..."
ros2 launch gazebo_ros gazebo.launch.py gui:=false world:=install/drone_rl/share/drone_rl/worlds/exploration_world.world &
GAZEBO_PID=$!
sleep 5

# Wait for Gazebo to be ready
echo "Waiting for Gazebo to initialize..."
timeout 30 bash -c 'until ros2 service list | grep -q "/gazebo/spawn_entity"; do sleep 1; done'

if ! ros2 service list | grep -q "/gazebo/spawn_entity"; then
    echo "ERROR: Gazebo failed to start properly"
    exit 1
fi

# Spawn robot entity in Gazebo
echo "Spawning robot in Gazebo..."
ros2 run gazebo_ros spawn_entity.py -file install/drone_rl/share/drone_rl/urdf/drone_simple.urdf -entity drone -x 0 -y 0 -z 0.1 &
SPAWN_PID=$!
sleep 5

# Start robot state publisher  
echo "Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat install/drone_rl/share/drone_rl/urdf/drone_simple.urdf)" -p use_sim_time:=true &
RSP_PID=$!
sleep 3

# Start SLAM
echo "Starting SLAM toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    -p use_sim_time:=true \
    -p base_frame:=base_link \
    -p odom_frame:=odom \
    -p map_frame:=map \
    -p scan_topic:=/scan \
    -p resolution:=0.05 \
    -p max_laser_range:=10.0 &
SLAM_PID=$!
sleep 3

# Start Q-learning agent
echo "Starting Q-learning exploration agent..."
ros2 run drone_rl q_learning_agent &
QLEARN_PID=$!
sleep 2

# Start exploration metrics
echo "Starting exploration metrics..."
ros2 run drone_rl exploration_metrics &
METRICS_PID=$!

# Start Gazebo GUI client (optional - will try but may fail)
echo "Attempting to start Gazebo GUI client..."
timeout 10 gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so &
GUI_PID=$!

echo ""
echo "=== AUTONOMOUS DRONE EXPLORATION SYSTEM ==="
echo "System launched successfully!"
echo "Process IDs:"
echo "- Gazebo server: $GAZEBO_PID"
echo "- Robot spawn: $SPAWN_PID" 
echo "- Robot state publisher: $RSP_PID"
echo "- SLAM: $SLAM_PID"
echo "- Q-learning agent: $QLEARN_PID"
echo "- Metrics: $METRICS_PID"
echo "- GUI client: $GUI_PID"
echo ""
echo "Check status with: ros2 topic list"
echo "Monitor cmd_vel: ros2 topic echo /cmd_vel"
echo "Monitor scan: ros2 topic echo /scan"
echo "Monitor odom: ros2 topic echo /odom"
echo "Stop system: pkill -f \"gzserver\|gzclient\|gazebo\|robot_state_publisher\|slam_toolbox\|q_learning\""
echo ""

# Wait for user input or run indefinitely
wait