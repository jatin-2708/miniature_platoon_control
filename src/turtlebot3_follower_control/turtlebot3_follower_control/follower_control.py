#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class EnhancedFollowerControl(Node):
    def __init__(self):
        super().__init__("enhanced_follower_control")
        
        # Parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('leader_odom_topic', '/tb0/odom'),
                ('follower_odom_topic', '/tb1/odom'),
                ('follower_cmd_vel_topic', '/tb1/cmd_vel'),
                ('desired_distance', 1.0),
                ('safety_distance', 0.5),
                ('lane_width', 0.6),
                ('max_speed', 0.22)
            ])
        
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # ... keep existing odom subscribers ...
        
        # Control variables
        self.obstacle_detected = False
        self.evasion_direction = None
        self.lane_offset = 0.0
        
        # PID parameters
        self.kp = 0.8
        self.ki = 0.001
        self.kd = 0.2
        self.integral = 0.0
        self.prev_error = 0.0

    def scan_callback(self, msg):
        """Process LIDAR data for obstacle detection"""
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        
        # Analyze front sector (30 degree cone)
        front_scan = np.concatenate((ranges[-30:], ranges[:30]))
        self.obstacle_detected = np.any(front_scan < self.get_parameter('safety_distance').value)
        
        if self.obstacle_detected:
            left_scan = ranges[30:90]
            right_scan = ranges[270:330]
            self.evasion_direction = 'left' if np.mean(left_scan) > np.mean(right_scan) else 'right'
            self.initiate_evasion()
        else:
            self.evasion_direction = None

    def initiate_evasion(self):
        """Execute lane change maneuver"""
        target_offset = self.get_parameter('lane_width').value
        if self.evasion_direction == 'left':
            self.lane_offset += target_offset
        else:
            self.lane_offset -= target_offset
            
        self.get_logger().warning(f"Evading {self.evasion_direction} to offset {self.lane_offset}")

    def calculate_target_position(self):
        """Adjust target based on lane offset and leader position"""
        base_x = self.leader_pos.x - self.get_parameter('desired_distance').value
        target_y = self.leader_pos.y + self.lane_offset
        
        # Smooth return to original lane
        if not self.obstacle_detected and abs(self.lane_offset) > 0.01:
            self.lane_offset *= 0.95  # Damping factor
            
        return (base_x, target_y)

    def control_loop(self):
        """Enhanced control with obstacle awareness"""
        if self.obstacle_detected:
            self.handle_obstacle_avoidance()
        else:
            super().control_loop()

    def handle_obstacle_avoidance(self):
        """Adaptive control during obstacle evasion"""
        target_x, target_y = self.calculate_target_position()
        
        dx = target_x - self.follower_pos.x
        dy = target_y - self.follower_pos.y
        
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        
        # Enhanced PID control with obstacle compensation
        error = distance - self.get_parameter('desired_distance').value
        self.integral += error
        derivative = error - self.prev_error
        
        cmd = Twist()
        cmd.linear.x = self.kp * error + self.ki * self.integral + self.kd * derivative
        cmd.angular.z = 1.5 * self.normalize_angle(bearing - self.follower_orientation)
        
        # Apply safety limits
        cmd.linear.x = np.clip(cmd.linear.x, -0.5*self.max_speed, self.max_speed)
        self.cmd_vel_pub.publish(cmd)
