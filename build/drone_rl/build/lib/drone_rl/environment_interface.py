#!/usr/bin/env python3
"""
Environment Interface for Drone RL
Provides state representation and environment interaction utilities
"""

import numpy as np
import math
from typing import Tuple, Optional, List, Dict
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class EnvironmentInterface(Node):
    """
    Interface between the RL agent and the ROS2 environment
    Handles sensor data processing and state representation
    """
    
    def __init__(self):
        super().__init__('environment_interface')
        
        # Environment parameters
        self.map_resolution = 0.05  # meters per pixel
        self.sensor_range = 10.0    # maximum sensor range
        self.safety_distance = 0.3  # minimum distance to obstacles
        
        # State representation
        self.occupancy_grid = None
        self.current_pose = None
        self.laser_scan = None
        
        # Exploration tracking
        self.explored_cells = set()
        self.frontier_cells = set()
        
        # QoS settings for map data
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            map_qos
        )
        
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # TF2 for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Environment Interface initialized")
    
    def map_callback(self, msg: OccupancyGrid):
        """Process occupancy grid from RTAB-Map"""
        self.occupancy_grid = msg
        self.update_exploration_data()
    
    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan data"""
        self.laser_scan = msg
    
    def odom_callback(self, msg: Odometry):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
    
    def update_exploration_data(self):
        """Update exploration tracking data"""
        if self.occupancy_grid is None:
            return
            
        grid_data = np.array(self.occupancy_grid.data).reshape(
            self.occupancy_grid.info.height,
            self.occupancy_grid.info.width
        )
        
        # Find known free cells (value 0)
        free_cells = np.where(grid_data == 0)
        self.explored_cells = set(zip(free_cells[0], free_cells[1]))
        
        # Find frontier cells (unknown cells adjacent to known cells)
        self.frontier_cells = self.find_frontier_cells(grid_data)
    
    def find_frontier_cells(self, grid_data: np.ndarray) -> set:
        """Find frontier cells for exploration"""
        frontiers = set()
        height, width = grid_data.shape
        
        # Define 8-connectivity neighbors
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), 
                    (0, 1), (1, -1), (1, 0), (1, 1)]
        
        for y in range(height):
            for x in range(width):
                if grid_data[y, x] == -1:  # Unknown cell
                    # Check if adjacent to known free space
                    for dy, dx in neighbors:
                        ny, nx = y + dy, x + dx
                        if (0 <= ny < height and 0 <= nx < width and 
                            grid_data[ny, nx] == 0):
                            frontiers.add((y, x))
                            break
        
        return frontiers
    
    def get_lidar_state(self, n_sectors: int = 8) -> Optional[np.ndarray]:
        """Get discretized LiDAR state representation"""
        if self.laser_scan is None:
            return None
            
        ranges = np.array(self.laser_scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.laser_scan.range_max)
        
        # Divide scan into sectors
        sector_size = len(ranges) // n_sectors
        sector_distances = []
        
        for i in range(n_sectors):
            start_idx = i * sector_size
            end_idx = (i + 1) * sector_size if i < n_sectors - 1 else len(ranges)
            sector_min = np.min(ranges[start_idx:end_idx])
            sector_distances.append(sector_min)
        
        return np.array(sector_distances)
    
    def get_pose_2d(self) -> Optional[Tuple[float, float, float]]:
        """Get 2D pose (x, y, yaw) from current pose"""
        if self.current_pose is None:
            return None
            
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # Extract yaw from quaternion
        orientation = self.current_pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        )
        
        return (x, y, yaw)
    
    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        if self.occupancy_grid is None:
            return (0, 0)
            
        origin = self.occupancy_grid.info.origin
        resolution = self.occupancy_grid.info.resolution
        
        grid_x = int((world_x - origin.position.x) / resolution)
        grid_y = int((world_y - origin.position.y) / resolution)
        
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        if self.occupancy_grid is None:
            return (0.0, 0.0)
            
        origin = self.occupancy_grid.info.origin
        resolution = self.occupancy_grid.info.resolution
        
        world_x = grid_x * resolution + origin.position.x
        world_y = grid_y * resolution + origin.position.y
        
        return (world_x, world_y)
    
    def get_nearest_frontier(self) -> Optional[Tuple[float, float]]:
        """Get coordinates of nearest frontier cell"""
        if not self.frontier_cells or self.current_pose is None:
            return None
            
        pose_2d = self.get_pose_2d()
        if pose_2d is None:
            return None
            
        drone_x, drone_y, _ = pose_2d
        drone_grid = self.world_to_grid(drone_x, drone_y)
        
        min_distance = float('inf')
        nearest_frontier = None
        
        for frontier_y, frontier_x in self.frontier_cells:
            distance = math.sqrt(
                (frontier_x - drone_grid[0])**2 + 
                (frontier_y - drone_grid[1])**2
            )
            if distance < min_distance:
                min_distance = distance
                nearest_frontier = self.grid_to_world(frontier_x, frontier_y)
        
        return nearest_frontier
    
    def calculate_frontier_direction(self) -> int:
        """Calculate direction to nearest frontier (0-7 for 8 directions)"""
        frontier = self.get_nearest_frontier()
        pose_2d = self.get_pose_2d()
        
        if frontier is None or pose_2d is None:
            return 0  # Default forward
            
        drone_x, drone_y, drone_yaw = pose_2d
        frontier_x, frontier_y = frontier
        
        # Calculate relative angle
        dx = frontier_x - drone_x
        dy = frontier_y - drone_y
        target_angle = math.atan2(dy, dx)
        
        # Calculate relative angle from drone's current heading
        relative_angle = target_angle - drone_yaw
        
        # Normalize angle to [-pi, pi]
        relative_angle = math.atan2(math.sin(relative_angle), math.cos(relative_angle))
        
        # Convert to 8-direction index (0-7)
        direction = int((relative_angle + math.pi) / (2 * math.pi / 8)) % 8
        return direction
    
    def check_collision_risk(self, threshold: Optional[float] = None) -> bool:
        """Check if drone is at risk of collision"""
        if threshold is None:
            threshold = self.safety_distance
            
        if self.laser_scan is None:
            return False
            
        ranges = np.array(self.laser_scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.laser_scan.range_max)
        
        return np.min(ranges) < threshold
    
    def get_exploration_progress(self) -> float:
        """Get exploration progress as percentage"""
        if self.occupancy_grid is None:
            return 0.0
            
        grid_data = np.array(self.occupancy_grid.data)
        total_cells = len(grid_data)
        known_cells = np.sum((grid_data == 0) | (grid_data == 100))  # Free or occupied
        
        return (known_cells / total_cells) * 100.0 if total_cells > 0 else 0.0
    
    def get_environment_state(self) -> Optional[Dict]:
        """Get comprehensive environment state for RL agent"""
        if (self.laser_scan is None or self.current_pose is None or 
            self.occupancy_grid is None):
            return None
            
        lidar_state = self.get_lidar_state()
        pose_2d = self.get_pose_2d()
        frontier_direction = self.calculate_frontier_direction()
        
        if lidar_state is None or pose_2d is None:
            return None
            
        return {
            'lidar_sectors': lidar_state,
            'pose_2d': pose_2d,
            'frontier_direction': frontier_direction,
            'collision_risk': self.check_collision_risk(),
            'exploration_progress': self.get_exploration_progress(),
            'frontier_count': len(self.frontier_cells)
        }


def main(args=None):
    rclpy.init(args=args)
    env_interface = EnvironmentInterface()
    
    try:
        rclpy.spin(env_interface)
    except KeyboardInterrupt:
        env_interface.get_logger().info("Shutting down Environment Interface...")
    finally:
        env_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()