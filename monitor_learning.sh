#!/bin/bash
# Q-Learning Monitoring Script - Shows timing and progress

echo "🧠 Q-LEARNING AUTONOMOUS EXPLORATION MONITOR"
echo "============================================"
echo ""
echo "📚 LEARNING PHASES:"
echo "   🔄 Phase 1 (0-2 mins): Exploration & random actions"
echo "   🎯 Phase 2 (2-5 mins): Learning optimal paths"
echo "   🚀 Phase 3 (5+ mins): Efficient autonomous exploration"
echo ""

cd ~/drone_ws
source install/setup.bash

echo "⏱️ TIMING EXPECTATIONS:"
echo "   • First movement: 10-30 seconds (sensor data collection)"
echo "   • Random exploration: 1-3 minutes (learning environment)"
echo "   • Smart navigation: 3-10 minutes (optimized paths)"
echo "   • Full exploration: 10-30 minutes (complete mapping)"
echo ""

echo "🔍 MONITORING SYSTEM STATUS..."
echo ""

# Check if Q-learning agent is running
if ps aux | grep -q "q_learning_agent" && ! ps aux | grep "q_learning_agent" | grep -q "grep"; then
    echo "✅ Q-Learning Agent: RUNNING"
    QL_STATUS="ACTIVE"
else
    echo "❌ Q-Learning Agent: NOT RUNNING"
    QL_STATUS="INACTIVE"
fi

# Check if drone is in simulation
if ros2 topic echo /gazebo/model_states --once 2>/dev/null | grep -q "drone"; then
    echo "✅ Drone in Simulation: FOUND"
    DRONE_STATUS="ACTIVE"
else
    echo "❌ Drone in Simulation: NOT FOUND" 
    DRONE_STATUS="INACTIVE"
fi

# Check if sensors are active
if ros2 topic list 2>/dev/null | grep -q "/scan"; then
    echo "✅ LiDAR Sensor: ACTIVE"
else
    echo "❌ LiDAR Sensor: INACTIVE"
fi

echo ""

if [[ "$QL_STATUS" == "ACTIVE" && "$DRONE_STATUS" == "ACTIVE" ]]; then
    echo "🎯 AUTONOMOUS LEARNING IN PROGRESS!"
    echo ""
    echo "📊 Live Monitoring (Press Ctrl+C to stop):"
    echo ""
    
    COUNTER=0
    START_TIME=$(date +%s)
    
    while true; do
        CURRENT_TIME=$(date +%s)
        ELAPSED=$((CURRENT_TIME - START_TIME))
        MINS=$((ELAPSED / 60))
        SECS=$((ELAPSED % 60))
        
        # Check for movement commands
        CMD_VEL_DATA=$(timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null)
        
        if [[ -n "$CMD_VEL_DATA" ]]; then
            LINEAR_X=$(echo "$CMD_VEL_DATA" | grep -A 1 "linear:" | grep "x:" | awk '{print $2}' | head -1)
            ANGULAR_Z=$(echo "$CMD_VEL_DATA" | grep -A 3 "angular:" | grep "z:" | awk '{print $2}' | head -1)
            
            if [[ "$LINEAR_X" != "0.0" ]] || [[ "$ANGULAR_Z" != "0.0" ]]; then
                STATUS="🚁 MOVING - Linear: $LINEAR_X, Angular: $ANGULAR_Z"
            else
                STATUS="⏸️  STATIONARY - Analyzing sensor data"
            fi
        else
            STATUS="📡 Waiting for sensor data..."
        fi
        
        printf "\r⏱️  Runtime: %02d:%02d | %s" $MINS $SECS "$STATUS"
        
        sleep 2
        COUNTER=$((COUNTER + 1))
        
        # Provide learning phase feedback
        if [[ $ELAPSED -eq 60 ]]; then
            echo ""
            echo "🔄 Entering Phase 2: Learning optimal exploration strategies..."
        elif [[ $ELAPSED -eq 300 ]]; then
            echo ""
            echo "🎯 Entering Phase 3: Efficient autonomous navigation!"
        fi
    done
    
else
    echo "❌ SYSTEM NOT READY FOR MONITORING"
    echo ""
    echo "🔧 Required actions:"
    if [[ "$DRONE_STATUS" == "INACTIVE" ]]; then
        echo "   1. Start Gazebo with drone spawned"
    fi
    if [[ "$QL_STATUS" == "INACTIVE" ]]; then
        echo "   2. Start Q-learning agent: install/drone_rl/bin/q_learning_agent &"
    fi
    echo ""
    echo "📋 Quick Start Command:"
    echo "   ./launch_final.sh"
fi

echo ""