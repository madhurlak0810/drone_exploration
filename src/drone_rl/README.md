# 🚁 Autonomous Drone Exploration with Reinforcement Learning

A comprehensive ROS2 package for autonomous drone exploration using Q-learning and SLAM mapping.

## 🌟 Features

- **Q-Learning Agent**: Autonomous exploration using reinforcement learning
- **SLAM Integration**: Real-time mapping with SLAM Toolbox
- **Gazebo Simulation**: Complete 3D simulation environment  
- **Performance Metrics**: Real-time exploration tracking and analysis
- **Modular Design**: Clean, extensible codebase

## 📋 System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **Gazebo**: Garden/Fortress

## 🚀 Quick Installation

### 1. Install Dependencies

```bash
# Install ROS2 Humble and required packages
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs \
  ros-humble-slam-toolbox ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher ros-humble-xacro \
  python3-colcon-common-extensions python3-numpy python3-matplotlib

# Optional: Install RTABMap for advanced SLAM (requires additional setup)
# sudo apt install ros-humble-rtabmap-ros
```

### 2. Build Package

```bash
# Navigate to workspace and build
cd ~/drone_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_rl
source install/setup.bash
```

### 3. Launch Simulation

```bash
# Launch the complete exploration environment
ros2 launch drone_rl sim_launch.py

# Or run headless for better performance
ros2 launch drone_rl sim_launch.py headless:=true rviz:=false
```

## 📊 Monitor Performance

```bash
# Real-time metrics
ros2 topic echo /exploration_metrics

# View saved metrics and plots
ls ~/drone_ws/metrics/
```

## 🎮 Manual Control (Optional)

```bash
# Install teleop tools
sudo apt install ros-humble-teleop-twist-keyboard

# Manual control for testing
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 🏗️ Architecture

```
drone_rl/
├── drone_rl/
│   ├── q_learning_agent.py      # Main RL agent
│   ├── environment_interface.py  # Environment state processing
│   └── exploration_metrics.py    # Performance tracking
├── launch/
│   └── sim_launch.py            # Complete launch configuration
├── config/
│   ├── drone_config.yaml       # Agent parameters
│   └── drone_exploration.rviz   # RViz configuration
├── urdf/
│   └── drone.urdf.xacro         # Robot description
├── worlds/
│   └── exploration_world.world  # Gazebo simulation world
└── scripts/
    └── quick_start.sh           # Quick setup script
```

## ⚙️ Configuration

Edit `config/drone_config.yaml` to customize:

- **Q-Learning parameters**: learning rate, epsilon, rewards
- **Environment settings**: sensor ranges, safety distances  
- **Control parameters**: velocities, update frequencies

## 📈 Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `learning_rate` | 0.1 | Q-learning learning rate |
| `epsilon` | 0.3 | Exploration vs exploitation rate |
| `safety_distance` | 0.3m | Minimum obstacle distance |
| `max_linear_velocity` | 0.5 m/s | Maximum forward speed |

## 🔄 Experiment Workflow

1. **Launch**: Start simulation with `ros2 launch drone_rl sim_launch.py`
2. **Monitor**: Watch live mapping in RViz and metrics in terminal
3. **Analyze**: Review saved metrics in `~/drone_ws/metrics/`
4. **Iterate**: Adjust parameters and repeat

## 📊 Metrics Tracked

- **Coverage Percentage**: Amount of environment mapped
- **Exploration Efficiency**: Coverage per unit time
- **Collision Rate**: Safety performance
- **Q-Learning Progress**: Reward accumulation and epsilon decay

## 🔧 Troubleshooting

### Common Issues

**Q-table not saving**: Check file permissions in `~/drone_ws/`

**SLAM not working**: Ensure topics are properly remapped:
```bash
ros2 topic list | grep -E "(scan|odom|map)"
```

**Poor exploration**: Adjust reward function in `drone_config.yaml`

**Gazebo crashes**: Reduce physics update rate or run headless

### Performance Tips

- Run headless (`headless:=true`) for better performance
- Adjust `control_frequency` based on hardware capabilities
- Monitor CPU/memory usage during long experiments

## 🎯 Future Enhancements

- [ ] **PPO Implementation**: Upgrade to continuous action space
- [ ] **3D Exploration**: Add altitude control for true 3D mapping
- [ ] **Multi-Agent**: Support for multiple drones
- [ ] **Real Robot**: Hardware deployment scripts
- [ ] **Advanced Rewards**: Sophisticated reward shaping

## 📝 Publications & References

This implementation is based on principles from:
- Q-Learning for robotics navigation
- SLAM Toolbox for real-time mapping
- Gazebo simulation best practices

## 🤝 Contributing

Feel free to submit issues, feature requests, and pull requests!

## 📄 License

MIT License - see LICENSE file for details

---

**Happy Exploring!** 🚁✨

For questions or support, please open an issue in the repository.