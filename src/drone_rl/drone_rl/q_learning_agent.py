#!/usr/bin/env python3
"""
Q-Learning Agent for Autonomous Drone Exploration
Integrates with RTAB-Map for SLAM and implements reward-based learning for exploration
"""

import numpy as np
import json
import os
from typing import Tuple, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32MultiArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import math
import time

class QLearningAgent(Node):
    def __init__(self):
        super().__init__('q_learning_agent')
        
        # Q-Learning Parameters
        self.learning_rate = 0.1
        self.discount_factor = 0.95
        self.epsilon = 0.3  # exploration rate
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01
        
        # State and Action Space
        self.n_distance_states = 10  # discretized LiDAR distances
        self.n_goal_states = 8       # directions to unexplored areas
        self.n_actions = 6           # movement actions
        
        # Initialize Q-table
        total_states = self.n_distance_states * self.n_goal_states
        self.q_table = np.zeros((total_states, self.n_actions))
        
        # State tracking
        self.current_state = None
        self.previous_state = None
        self.previous_action = None
        self.last_pose = None
        self.exploration_start_time = time.time()
        
        # Exploration metrics
        self.total_reward = 0.0
        self.step_count = 0
        self.collision_count = 0
        self.new_areas_explored = 0
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/exploration_metrics', 10)
        
        # Subscribers with appropriate QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # TF2 for pose transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Environment state
        self.current_scan = None
        self.occupancy_grid = None
        self.current_pose = None
        self.previous_position = None
        
        # Control timer - slower for stable navigation
        self.control_timer = self.create_timer(0.5, self.control_loop)  # 2 Hz
        
        # Save Q-table periodically
        self.save_timer = self.create_timer(30.0, self.save_q_table)  # Every 30 seconds
        
        # Load existing Q-table if available
        self.load_q_table()
        
        self.get_logger().info("Q-Learning Agent initialized with RTAB-Map integration")

    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan data"""
        self.current_scan = msg
        
    def map_callback(self, msg: OccupancyGrid):
        """Process RTAB-Map occupancy grid"""
        self.occupancy_grid = msg
        
    def odom_callback(self, msg: Odometry):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
        
    def control_loop(self):
        """Main control loop for Q-learning"""
        # Wait for odometry data
        if self.current_scan is None or self.current_pose is None:
            return
            
        # Get current state
        state = self.get_state()
        if state is None:
            return
            
        # Calculate reward if we have a previous state
        reward = 0.0
        if self.previous_state is not None and self.previous_action is not None:
            reward = self.calculate_reward()
            
            # Update Q-table
            self.update_q_table(self.previous_state, self.previous_action, reward, state)
            
        # Choose action (epsilon-greedy)
        action = self.choose_action(state)
        
        # Execute action
        self.execute_action(action)
        
        # Update state tracking
        self.previous_state = state
        self.previous_action = action
        self.current_state = state
        self.step_count += 1
        self.total_reward += reward
        
        # Publish metrics
        self.publish_metrics()
        
        # Decay epsilon
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def get_state(self) -> Optional[int]:
        """Convert sensor data to discrete state"""
        if self.current_scan is None:
            return None
            
        # Process LiDAR data - get minimum distance in front sectors
        ranges = np.array(self.current_scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.current_scan.range_max)
        
        # Front sector (±30 degrees)
        front_start = len(ranges) // 2 - len(ranges) // 12
        front_end = len(ranges) // 2 + len(ranges) // 12
        front_distances = ranges[front_start:front_end]
        min_front_distance = np.min(front_distances)
        
        # Discretize distance (0-9 based on proximity)
        distance_state = min(int(min_front_distance / 0.5), self.n_distance_states - 1)
        
        # Get goal direction state (direction to nearest unexplored area)
        goal_state = self.get_goal_direction_state()
        
        # Combine states
        combined_state = distance_state * self.n_goal_states + goal_state
        return combined_state

    def get_goal_direction_state(self) -> int:
        """Determine direction to nearest unexplored area"""
        if self.occupancy_grid is None:
            return 0  # Default forward direction
            
        # Simple implementation: find unexplored cells and determine direction
        # This is a simplified version - can be enhanced with frontier detection
        grid = np.array(self.occupancy_grid.data).reshape(
            self.occupancy_grid.info.height, 
            self.occupancy_grid.info.width
        )
        
        # Find unknown cells (value -1)
        unknown_cells = np.where(grid == -1)
        
        if len(unknown_cells[0]) == 0:
            return 0  # No unexplored areas found
            
        # Get drone position in grid coordinates
        if self.current_pose is None:
            return 0
            
        # Convert pose to grid coordinates
        resolution = self.occupancy_grid.info.resolution
        origin = self.occupancy_grid.info.origin
        
        grid_x = int((self.current_pose.position.x - origin.position.x) / resolution)
        grid_y = int((self.current_pose.position.y - origin.position.y) / resolution)
        
        # Find closest unknown cell
        distances = np.sqrt((unknown_cells[1] - grid_x)**2 + (unknown_cells[0] - grid_y)**2)
        closest_idx = np.argmin(distances)
        
        target_x = unknown_cells[1][closest_idx]
        target_y = unknown_cells[0][closest_idx]
        
        # Calculate angle to target (8 directional states)
        dx = target_x - grid_x
        dy = target_y - grid_y
        angle = math.atan2(dy, dx)
        
        # Convert to discrete direction (0-7)
        direction = int((angle + math.pi) / (2 * math.pi / 8)) % 8
        return direction

    def choose_action(self, state: int) -> int:
        """Choose action using epsilon-greedy strategy"""
        if np.random.random() < self.epsilon:
            # Exploration: random action
            return np.random.randint(0, self.n_actions)
        else:
            # Exploitation: best known action
            return int(np.argmax(self.q_table[state]))

    def execute_action(self, action: int):
        """Execute the chosen action with wall avoidance and escape behaviors"""
        twist = Twist()
        
        # Check for obstacles with more forgiving thresholds
        obstacle_ahead = False
        obstacle_left = False
        obstacle_right = False
        min_front_distance = float('inf')
        min_left_distance = float('inf')
        min_right_distance = float('inf')
        
        if self.current_scan is not None:
            ranges = np.array(self.current_scan.ranges)
            ranges = np.where(np.isfinite(ranges), ranges, self.current_scan.range_max)
            
            # Use wider sectors for more stable detection
            sector_size = len(ranges) // 8  # 45 degrees per sector
            center = len(ranges) // 2
            
            # Front sector (±22.5 degrees) - wider for stability
            front_start = center - sector_size//2
            front_end = center + sector_size//2
            front_ranges = ranges[front_start:front_end]
            valid_front = front_ranges[front_ranges > 0.3]  # Filter robot body
            if len(valid_front) > 0:
                min_front_distance = np.min(valid_front)
                obstacle_ahead = min_front_distance < 0.7  # More forgiving threshold
            
            # Left sector (45-90 degrees)
            left_ranges = ranges[center + sector_size:center + 2*sector_size]
            valid_left = left_ranges[left_ranges > 0.3]
            if len(valid_left) > 0:
                min_left_distance = np.min(valid_left)
                obstacle_left = min_left_distance < 0.6  # More forgiving
                
            # Right sector (-45 to -90 degrees)
            right_ranges = ranges[center - 2*sector_size:center - sector_size]
            valid_right = right_ranges[right_ranges > 0.3]
            if len(valid_right) > 0:
                min_right_distance = np.min(valid_right)
                obstacle_right = min_right_distance < 0.6  # More forgiving
        
        # Anti-stuck obstacle avoidance with progressive responses
        if obstacle_ahead:
            if min_front_distance < 0.4:  # True emergency - very close
                # Emergency stop and turn
                twist.linear.x = 0.0
                if min_left_distance > min_right_distance:
                    twist.angular.z = 0.8  # Sharp turn left
                else:
                    twist.angular.z = -0.8  # Sharp turn right
                self.get_logger().warn(f"Emergency stop: obstacle at {min_front_distance:.2f}m")
            elif min_front_distance < 0.6:  # Close obstacle - slow and turn
                # Slow down and turn toward clearer side
                twist.linear.x = 0.05  # Very slow forward
                if not obstacle_left and min_left_distance > 1.2:
                    twist.angular.z = 0.5  # Turn left toward clear space
                elif not obstacle_right and min_right_distance > 1.2:
                    twist.angular.z = -0.5  # Turn right toward clear space
                else:
                    # No clear space - turn in place toward better option
                    twist.linear.x = 0.0
                    if min_left_distance > min_right_distance:
                        twist.angular.z = 0.6
                    else:
                        twist.angular.z = -0.6
            else:
                # Distant obstacle - gentle avoidance
                twist.linear.x = 0.15  # Moderate speed
                if not obstacle_left:
                    twist.angular.z = 0.3  # Gentle left turn
                elif not obstacle_right:
                    twist.angular.z = -0.3  # Gentle right turn
        else:
            # Normal action execution when path is clear
            if action == 0:    # Move forward
                twist.linear.x = 0.2
            elif action == 1:  # Turn left and move
                twist.linear.x = 0.15
                twist.angular.z = 0.5
            elif action == 2:  # Turn right and move
                twist.linear.x = 0.15
                twist.angular.z = -0.5
            elif action == 3:  # Turn left sharp
                twist.angular.z = 0.7
            elif action == 4:  # Turn right sharp
                twist.angular.z = -0.7
            elif action == 5:  # Stop/hover
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
        self.cmd_pub.publish(twist)

    def calculate_reward(self) -> float:
        """Calculate reward based on current state and action"""
        reward = 0.0
        
        # Skip collision detection for first 10 steps to allow system stabilization
        if self.step_count < 10:
            return reward
        
        # Check for collision using sectored analysis instead of global minimum
        if self.current_scan is not None:
            ranges = np.array(self.current_scan.ranges)
            ranges = np.where(np.isfinite(ranges), ranges, self.current_scan.range_max)
            
            # Only check front 90 degrees for collision detection (balanced approach)
            # Assuming 360-degree scan, check front sector
            sixth_size = len(ranges) // 8
            center = len(ranges) // 2
            front_ranges = ranges[center-sixth_size:center+sixth_size]  # Front 90 degrees
            
            # Filter out robot body and ground reflections (< 0.3m)
            valid_front_ranges = front_ranges[front_ranges > 0.3]
            
            if len(valid_front_ranges) > 0:
                min_front_distance = np.min(valid_front_ranges)
                
                if min_front_distance < 0.4:  # True collision threshold (matches emergency stop)
                    reward -= 10.0  # Penalty for collision
                    self.collision_count += 1
                elif min_front_distance < 0.7:  # Near collision threshold (matches obstacle detection)
                    reward -= 2.0   # Penalty for being too close
                
        # Reward for exploration (new area discovered)
        if self.occupancy_grid is not None and self.previous_position is not None and self.current_pose is not None:
            current_pos = (self.current_pose.position.x, self.current_pose.position.y)
            distance_moved = math.sqrt(
                (current_pos[0] - self.previous_position[0])**2 +
                (current_pos[1] - self.previous_position[1])**2
            )
            
            if distance_moved > 0.1:  # Moved significantly
                reward += 3.0  # Good reward for movement
                
                # Check if moved to unexplored area
                if self.moved_to_unexplored_area():
                    reward += 20.0  # High reward for exploration
                    self.new_areas_explored += 1
            elif distance_moved < 0.05:  # Very little movement - might be stuck
                reward -= 5.0  # Penalty for being stuck
                
        # Penalty for being too close to walls (encourages staying in open space)
        if self.current_scan is not None:
            ranges = np.array(self.current_scan.ranges)
            ranges = np.where(np.isfinite(ranges), ranges, self.current_scan.range_max)
            avg_distance = np.mean(ranges[ranges > 0.35])  # Average distance to obstacles
            if avg_distance < 1.5:  # Close to walls
                reward -= 1.0
                
        # Small penalty for staying idle (reduced)
        if self.previous_action == 5:  # Stop action
            reward -= 1.0
            
        # Update previous position
        if self.current_pose is not None:
            self.previous_position = (self.current_pose.position.x, self.current_pose.position.y)
            
        return reward

    def moved_to_unexplored_area(self) -> bool:
        """Check if drone moved to a previously unexplored area"""
        if self.occupancy_grid is None or self.current_pose is None:
            return False
            
        # Convert current position to grid coordinates
        resolution = self.occupancy_grid.info.resolution
        origin = self.occupancy_grid.info.origin
        
        grid_x = int((self.current_pose.position.x - origin.position.x) / resolution)
        grid_y = int((self.current_pose.position.y - origin.position.y) / resolution)
        
        # Check if position is within grid bounds
        if (0 <= grid_x < self.occupancy_grid.info.width and 
            0 <= grid_y < self.occupancy_grid.info.height):
            
            grid = np.array(self.occupancy_grid.data).reshape(
                self.occupancy_grid.info.height, 
                self.occupancy_grid.info.width
            )
            
            # Check surrounding area for unknown cells
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    check_x, check_y = grid_x + dx, grid_y + dy
                    if (0 <= check_x < self.occupancy_grid.info.width and 
                        0 <= check_y < self.occupancy_grid.info.height):
                        if grid[check_y, check_x] == -1:  # Unknown cell
                            return True
                            
        return False

    def update_q_table(self, state: int, action: int, reward: float, next_state: int):
        """Update Q-table using Q-learning algorithm"""
        current_q = self.q_table[state, action]
        max_next_q = np.max(self.q_table[next_state])
        
        new_q = current_q + self.learning_rate * (
            reward + self.discount_factor * max_next_q - current_q
        )
        
        self.q_table[state, action] = new_q

    def publish_metrics(self):
        """Publish exploration metrics"""
        metrics = Float32MultiArray()
        metrics.data = [
            float(self.total_reward),
            float(self.step_count),
            float(self.collision_count),
            float(self.new_areas_explored),
            float(self.epsilon)
        ]
        self.metrics_pub.publish(metrics)

    def save_q_table(self):
        """Save Q-table to file"""
        try:
            save_path = os.path.expanduser("~/drone_ws/q_table.npy")
            np.save(save_path, self.q_table)
            self.get_logger().info(f"Q-table saved to {save_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save Q-table: {e}")

    def load_q_table(self):
        """Load Q-table from file if it exists"""
        try:
            load_path = os.path.expanduser("~/drone_ws/q_table.npy")
            if os.path.exists(load_path):
                self.q_table = np.load(load_path)
                self.get_logger().info(f"Q-table loaded from {load_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not load Q-table: {e}")

def main(args=None):
    rclpy.init(args=args)
    agent = QLearningAgent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info("Shutting down Q-Learning Agent...")
    finally:
        agent.save_q_table()  # Save Q-table on shutdown
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
