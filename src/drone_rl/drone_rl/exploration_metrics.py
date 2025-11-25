#!/usr/bin/env python3
"""
Exploration Metrics for Drone RL
Tracks and analyzes exploration performance
"""

import numpy as np
import json
import time
from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import os


class ExplorationMetrics(Node):
    """
    Node for tracking and analyzing exploration performance metrics
    """
    
    def __init__(self):
        super().__init__('exploration_metrics')
        
        # Metrics tracking
        self.start_time = time.time()
        self.metrics_history = []
        self.coverage_history = []
        self.velocity_history = []
        
        # Coverage calculation
        self.initial_unknown_cells = 0
        self.total_map_cells = 0
        self.current_coverage = 0.0
        
        # Performance metrics
        self.total_distance_traveled = 0.0
        self.previous_pose = None
        self.collision_count = 0
        self.exploration_efficiency = 0.0
        
        # QoS for map data
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Subscribers
        self.create_subscription(
            Float32MultiArray,
            '/exploration_metrics',
            self.metrics_callback,
            10
        )
        
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )
        
        # Timer for periodic analysis
        self.analysis_timer = self.create_timer(10.0, self.analyze_performance)
        
        # Timer for saving metrics
        self.save_timer = self.create_timer(60.0, self.save_metrics)
        
        self.get_logger().info("Exploration Metrics node initialized")
    
    def metrics_callback(self, msg: Float32MultiArray):
        """Process metrics from Q-learning agent"""
        if len(msg.data) >= 5:
            timestamp = time.time() - self.start_time
            metrics = {
                'timestamp': timestamp,
                'total_reward': msg.data[0],
                'step_count': msg.data[1],
                'collision_count': msg.data[2],
                'new_areas_explored': msg.data[3],
                'epsilon': msg.data[4]
            }
            self.metrics_history.append(metrics)
            self.collision_count = int(msg.data[2])
    
    def map_callback(self, msg: OccupancyGrid):
        """Process occupancy grid for coverage calculation"""
        grid_data = np.array(msg.data)
        
        # Calculate coverage metrics
        unknown_cells = np.sum(grid_data == -1)
        free_cells = np.sum(grid_data == 0)
        occupied_cells = np.sum(grid_data == 100)
        
        self.total_map_cells = len(grid_data)
        known_cells = free_cells + occupied_cells
        
        # Initialize baseline on first map
        if self.initial_unknown_cells == 0:
            self.initial_unknown_cells = unknown_cells
            self.get_logger().info(f"Initial map analysis: {unknown_cells} unknown, {free_cells} free, {occupied_cells} occupied cells")
            
        # Calculate coverage percentage
        explored_cells = 0  # Initialize explored_cells
        if self.initial_unknown_cells > 0:
            explored_cells = max(0, self.initial_unknown_cells - unknown_cells)
            self.current_coverage = (explored_cells / self.initial_unknown_cells) * 100.0
            
            # Alternative coverage based on known cells (more responsive)
            known_cell_coverage = (known_cells / self.total_map_cells) * 100.0
            
            # Use the higher of the two coverage measures for more responsive feedback
            self.current_coverage = max(self.current_coverage, known_cell_coverage)
        else:
            # If no initial unknown cells, calculate based on known cells vs total
            if self.total_map_cells > 0:
                self.current_coverage = (known_cells / self.total_map_cells) * 100.0
                
        # Debug logging every 20 map updates instead of 50
        if len(self.coverage_history) % 20 == 0:
            self.get_logger().info(f"Coverage debug: {explored_cells} explored of {self.initial_unknown_cells} initial, Known cells: {known_cells}/{self.total_map_cells}, Current: {self.current_coverage:.1f}%")
        
        # Store coverage history
        timestamp = time.time() - self.start_time
        self.coverage_history.append({
            'timestamp': timestamp,
            'coverage_percentage': self.current_coverage,
            'unknown_cells': unknown_cells,
            'free_cells': free_cells,
            'occupied_cells': occupied_cells
        })
    
    def velocity_callback(self, msg: Twist):
        """Track velocity commands for distance calculation"""
        timestamp = time.time() - self.start_time
        velocity_data = {
            'timestamp': timestamp,
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z
        }
        self.velocity_history.append(velocity_data)
        
        # Estimate distance traveled (simple integration)
        if len(self.velocity_history) > 1:
            dt = velocity_data['timestamp'] - self.velocity_history[-2]['timestamp']
            distance_increment = abs(msg.linear.x) * dt
            self.total_distance_traveled += distance_increment
    
    def analyze_performance(self):
        """Analyze and log performance metrics"""
        if not self.metrics_history:
            return
            
        current_time = time.time() - self.start_time
        latest_metrics = self.metrics_history[-1]
        
        # Calculate exploration efficiency (coverage per unit time)
        if current_time > 0:
            self.exploration_efficiency = self.current_coverage / (current_time / 60.0)  # % per minute
        
        # Calculate collision rate
        collision_rate = self.collision_count / max(latest_metrics['step_count'], 1) * 100.0
        
        # Log performance summary
        self.get_logger().info(
            f"Performance Summary:\n"
            f"  Time: {current_time:.1f}s\n"
            f"  Coverage: {self.current_coverage:.1f}%\n"
            f"  Efficiency: {self.exploration_efficiency:.2f}%/min\n"
            f"  Distance: {self.total_distance_traveled:.1f}m\n"
            f"  Collisions: {self.collision_count}\n"
            f"  Collision Rate: {collision_rate:.2f}%\n"
            f"  Total Reward: {latest_metrics['total_reward']:.1f}\n"
            f"  Epsilon: {latest_metrics['epsilon']:.3f}"
        )
    
    def save_metrics(self):
        """Save metrics to files for analysis"""
        try:
            save_dir = os.path.expanduser("~/drone_ws/metrics")
            os.makedirs(save_dir, exist_ok=True)
            
            # Save raw metrics
            metrics_file = os.path.join(save_dir, "exploration_metrics.json")
            with open(metrics_file, 'w') as f:
                json.dump({
                    'metrics_history': self.metrics_history,
                    'coverage_history': self.coverage_history,
                    'velocity_history': self.velocity_history[-100:],  # Keep last 100 entries
                    'summary': {
                        'total_distance': self.total_distance_traveled,
                        'current_coverage': self.current_coverage,
                        'exploration_efficiency': self.exploration_efficiency,
                        'collision_count': self.collision_count
                    }
                }, f, indent=2)
            
            self.get_logger().info(f"Metrics saved to {metrics_file}")
            
            # Generate plots if matplotlib is available
            self.generate_plots(save_dir)
            
        except Exception as e:
            self.get_logger().error(f"Failed to save metrics: {e}")
    
    def generate_plots(self, save_dir: str):
        """Generate performance plots"""
        try:
            if not self.metrics_history or not self.coverage_history:
                return
                
            # Coverage over time plot
            plt.figure(figsize=(12, 8))
            
            # Coverage subplot
            plt.subplot(2, 2, 1)
            timestamps = [entry['timestamp'] for entry in self.coverage_history]
            coverage = [entry['coverage_percentage'] for entry in self.coverage_history]
            plt.plot(timestamps, coverage, 'b-', linewidth=2)
            plt.xlabel('Time (seconds)')
            plt.ylabel('Coverage (%)')
            plt.title('Map Coverage Over Time')
            plt.grid(True)
            
            # Reward subplot
            plt.subplot(2, 2, 2)
            timestamps = [entry['timestamp'] for entry in self.metrics_history]
            rewards = [entry['total_reward'] for entry in self.metrics_history]
            plt.plot(timestamps, rewards, 'g-', linewidth=2)
            plt.xlabel('Time (seconds)')
            plt.ylabel('Total Reward')
            plt.title('Cumulative Reward Over Time')
            plt.grid(True)
            
            # Collision count subplot
            plt.subplot(2, 2, 3)
            collisions = [entry['collision_count'] for entry in self.metrics_history]
            plt.plot(timestamps, collisions, 'r-', linewidth=2)
            plt.xlabel('Time (seconds)')
            plt.ylabel('Collision Count')
            plt.title('Collisions Over Time')
            plt.grid(True)
            
            # Epsilon (exploration rate) subplot
            plt.subplot(2, 2, 4)
            epsilons = [entry['epsilon'] for entry in self.metrics_history]
            plt.plot(timestamps, epsilons, 'm-', linewidth=2)
            plt.xlabel('Time (seconds)')
            plt.ylabel('Epsilon (Exploration Rate)')
            plt.title('Exploration Rate Over Time')
            plt.grid(True)
            
            plt.tight_layout()
            plot_file = os.path.join(save_dir, "exploration_performance.png")
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            plt.close()
            
            self.get_logger().info(f"Performance plots saved to {plot_file}")
            
        except ImportError:
            self.get_logger().warn("matplotlib not available, skipping plot generation")
        except Exception as e:
            self.get_logger().error(f"Failed to generate plots: {e}")
    
    def get_performance_summary(self) -> Dict:
        """Get comprehensive performance summary"""
        if not self.metrics_history:
            return {}
            
        latest_metrics = self.metrics_history[-1]
        current_time = time.time() - self.start_time
        
        return {
            'runtime_seconds': current_time,
            'coverage_percentage': self.current_coverage,
            'exploration_efficiency_percent_per_minute': self.exploration_efficiency,
            'total_distance_meters': self.total_distance_traveled,
            'collision_count': self.collision_count,
            'collision_rate_percent': self.collision_count / max(latest_metrics['step_count'], 1) * 100.0,
            'total_reward': latest_metrics['total_reward'],
            'steps_taken': latest_metrics['step_count'],
            'current_epsilon': latest_metrics['epsilon'],
            'areas_explored': latest_metrics['new_areas_explored']
        }
    
    def reset_metrics(self):
        """Reset all metrics (useful for new experiments)"""
        self.start_time = time.time()
        self.metrics_history.clear()
        self.coverage_history.clear()
        self.velocity_history.clear()
        self.initial_unknown_cells = 0
        self.total_distance_traveled = 0.0
        self.collision_count = 0
        self.current_coverage = 0.0
        
        self.get_logger().info("Metrics reset for new experiment")


def main(args=None):
    rclpy.init(args=args)
    metrics_node = ExplorationMetrics()
    
    try:
        rclpy.spin(metrics_node)
    except KeyboardInterrupt:
        metrics_node.get_logger().info("Shutting down Exploration Metrics...")
        # Final save on shutdown
        metrics_node.save_metrics()
    finally:
        metrics_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()