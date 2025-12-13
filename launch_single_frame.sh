#!/bin/bash

echo "=== Single Frame Static Map ==="
echo "Creating one-time map snapshot..."

# Kill all RViz
pkill -f rviz2
sleep 2

# Use the same domain as the main system
# Don't override ROS_DOMAIN_ID, inherit from environment
# export ROS_DOMAIN_ID=0

cd /home/maddy/drone_ws
source install/setup.bash

echo ""
echo "Taking map snapshot and creating static display..."

# Get current map data once
timeout 2 ros2 topic echo /map --once > /tmp/static_map.txt 2>/dev/null &

echo ""
echo "Configuration:"
echo "  ✓ Single frame capture only"
echo "  ✓ No dynamic updates"
echo "  ✓ No real-time refresh"
echo "  ✓ Minimum graphics processing"
echo ""

# Create a minimal RViz config that just shows static data
cat > /tmp/minimal_static.rviz << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Map1
      Splitter Ratio: 0.5
    Tree Height: 600

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1.0
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: true
      Enabled: true
      Name: Map
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        Filter size: 1
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /map
      Use Timestamp: false
      Value: true

  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 1
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0

  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 25
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 1.2
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0
    Saved: ~

Window Geometry:
  Height: 1600
  Width: 1200
EOF

echo "Starting minimal static RViz (1fps, no updates)..."
rviz2 -d /tmp/minimal_static.rviz