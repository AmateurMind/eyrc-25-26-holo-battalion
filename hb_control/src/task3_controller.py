#!/usr/bin/env python3
"""
Task 3 Integrated Controller for Holo Battalion Theme.

This controller combines holonomic robot navigation with arm/gripper control
to perform pick-and-place operations on colored boxes.

Task 3 Objective:
- Navigate to colored boxes (red, green, blue)
- Pick up boxes using the arm/gripper system
- Transport boxes to corresponding drop zones
- Place boxes in the correct zones

Publishers:
    /forward_velocity_controller/commands (Float64MultiArray): Wheel velocities
    /arm_position_controller/commands (Float64MultiArray): Arm positions
    /gripper_position_controller/commands (Float64MultiArray): Gripper positions

Subscribers:
    /bot_pose (Poses2D): Current robot pose from perception
    /crates_pose (Poses2D): Box positions from perception
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from hb_interfaces.msg import Pose2D, Poses2D
import numpy as np
import math
import time
from enum import Enum


class RobotState(Enum):
    """State machine states for Task 3 operations."""
    IDLE = 0
    NAVIGATE_TO_BOX = 1
    APPROACH_BOX = 2
    PICK_BOX = 3
    NAVIGATE_TO_ZONE = 4
    PLACE_BOX = 5
    RETURN_HOME = 6
    COMPLETED = 7


class PID:
    """PID Controller class."""
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        
        # Integral term
        self.integral += error * dt
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        
        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Store previous error
        self.prev_error = error
        
        # Clamp output
        return np.clip(output, -self.max_out, self.max_out)
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class Task3Controller(Node):
    """Integrated controller for Task 3 pick-and-place operations."""
    
    def __init__(self):
        super().__init__('task3_controller')

        # ============================================
        # ROBOT PARAMETERS
        # ============================================
        self.max_vel = 5.0  # Maximum wheel velocity
        self.current_pose = None
        self.crate_poses = {}
        
        # ============================================
        # TASK 3 CONFIGURATION
        # ============================================
        
        # Drop zone positions (world coordinates in pixels/units)
        self.drop_zones = {
            'red': (1200, 800, 0),      # Red zone position (x, y, theta)
            'green': (1200, -800, 0),   # Green zone position
            'blue': (-1200, 0, 0)       # Blue zone position
        }
        
        # Box-to-zone mapping (which color box goes to which zone)
        self.box_zone_map = {
            'red': 'red',
            'green': 'green', 
            'blue': 'blue'
        }
        
        # Task queue: list of (box_id, color) to process
        self.task_queue = []
        self.current_task = None
        self.current_box_pose = None
        self.holding_box = False
        
        # State machine
        self.state = RobotState.IDLE
        self.home_position = (500, 500, 0)
        
        # Navigation thresholds
        self.position_threshold = 30  # Position error threshold (pixels)
        self.angle_threshold = 0.1    # Angle error threshold (radians)
        self.approach_distance = 100  # Distance to stop for picking

        # ============================================
        # ARM/GRIPPER PARAMETERS
        # ============================================
        self.lift_min = 0.0
        self.lift_max = 0.08
        self.extend_min = 0.0
        self.extend_max = 0.05
        self.gripper_open = 0.0
        self.gripper_close = -0.015
        
        self.current_lift = 0.0
        self.current_extend = 0.0
        self.current_gripper = 0.0

        # ============================================
        # PID CONTROLLERS
        # ============================================
        self.pid_params = {
            'x': {'kp': 0.005, 'ki': 0.0001, 'kd': 0.001, 'max_out': self.max_vel},
            'y': {'kp': 0.005, 'ki': 0.0001, 'kd': 0.001, 'max_out': self.max_vel},
            'theta': {'kp': 1.0, 'ki': 0.01, 'kd': 0.1, 'max_out': self.max_vel * 2}
        }
        
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # Timing
        self.last_time = self.get_clock().now()
        self.pick_place_timer = None
        self.pick_place_step = 0

        # ============================================
        # ROS 2 PUBLISHERS
        # ============================================
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10
        )
        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/arm_position_controller/commands', 10
        )
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_position_controller/commands', 10
        )

        # ============================================
        # ROS 2 SUBSCRIBERS
        # ============================================
        self.pose_sub = self.create_subscription(
            Poses2D, '/bot_pose', self.bot_pose_callback, 10
        )
        self.crate_sub = self.create_subscription(
            Poses2D, '/crates_pose', self.crate_pose_callback, 10
        )

        # ============================================
        # CONTROL LOOP TIMER
        # ============================================
        self.control_timer = self.create_timer(0.03, self.control_loop)

        self.get_logger().info('Task 3 Controller initialized')
        self.get_logger().info(f'Drop zones: {self.drop_zones}')

    def bot_pose_callback(self, msg):
        """Update current robot pose from perception."""
        for pose in msg.poses:
            # Assuming bot has a specific ID range (e.g., 0-9)
            if pose.id < 10:
                self.current_pose = (pose.x, pose.y, pose.w)
                break

    def crate_pose_callback(self, msg):
        """Update crate poses from perception."""
        for pose in msg.poses:
            # Store pose with ID
            self.crate_poses[pose.id] = {
                'x': pose.x,
                'y': pose.y,
                'theta': pose.w,
                'color': self.get_box_color(pose.id)
            }
        
        # Build task queue if empty
        if not self.task_queue and self.state == RobotState.IDLE:
            self.build_task_queue()

    def get_box_color(self, box_id):
        """Determine box color based on ID."""
        # Example mapping - adjust based on actual ArUco marker IDs
        if 10 <= box_id < 20:
            return 'red'
        elif 20 <= box_id < 30:
            return 'green'
        elif 30 <= box_id < 40:
            return 'blue'
        return 'unknown'

    def build_task_queue(self):
        """Build the task queue from detected crates."""
        for box_id, box_info in self.crate_poses.items():
            if box_info['color'] in self.box_zone_map:
                self.task_queue.append((box_id, box_info['color']))
        
        if self.task_queue:
            self.get_logger().info(f'Task queue built: {self.task_queue}')
            self.state = RobotState.NAVIGATE_TO_BOX
            self.current_task = self.task_queue.pop(0)

    def control_loop(self):
        """Main control loop - state machine."""
        if self.current_pose is None:
            return

        # Calculate dt
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        # State machine
        if self.state == RobotState.IDLE:
            self.stop_robot()
            
        elif self.state == RobotState.NAVIGATE_TO_BOX:
            self.navigate_to_box(dt)
            
        elif self.state == RobotState.APPROACH_BOX:
            self.approach_box(dt)
            
        elif self.state == RobotState.PICK_BOX:
            self.execute_pick()
            
        elif self.state == RobotState.NAVIGATE_TO_ZONE:
            self.navigate_to_zone(dt)
            
        elif self.state == RobotState.PLACE_BOX:
            self.execute_place()
            
        elif self.state == RobotState.RETURN_HOME:
            self.return_home(dt)
            
        elif self.state == RobotState.COMPLETED:
            self.stop_robot()
            self.get_logger().info('All tasks completed!')

        # Publish arm/gripper commands
        self.publish_arm_gripper()

    def navigate_to_box(self, dt):
        """Navigate towards target box."""
        if self.current_task is None:
            self.state = RobotState.COMPLETED
            return

        box_id, color = self.current_task
        
        if box_id not in self.crate_poses:
            self.get_logger().warn(f'Box {box_id} not found')
            return

        box_info = self.crate_poses[box_id]
        target_x, target_y = box_info['x'], box_info['y']
        
        # Calculate distance
        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.approach_distance:
            self.state = RobotState.APPROACH_BOX
            self.current_box_pose = (target_x, target_y, box_info['theta'])
            self.reset_pids()
            return
        
        # Navigate
        self.navigate_to_point(target_x, target_y, 0, dt)

    def approach_box(self, dt):
        """Approach box for picking."""
        if self.current_box_pose is None:
            return

        target_x, target_y, _ = self.current_box_pose
        
        # Calculate distance
        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.position_threshold:
            self.stop_robot()
            self.state = RobotState.PICK_BOX
            self.pick_place_step = 0
            return
        
        # Slow approach
        self.navigate_to_point(target_x, target_y, 0, dt, speed_factor=0.3)

    def execute_pick(self):
        """Execute pick sequence step by step."""
        if self.pick_place_step == 0:
            # Open gripper
            self.current_gripper = self.gripper_open
            self.pick_place_step = 1
            self.create_one_shot_timer(0.5)
            
        elif self.pick_place_step == 1:
            # Lower arm
            self.current_lift = self.lift_min
            self.pick_place_step = 2
            self.create_one_shot_timer(0.5)
            
        elif self.pick_place_step == 2:
            # Extend arm
            self.current_extend = self.extend_max
            self.pick_place_step = 3
            self.create_one_shot_timer(0.5)
            
        elif self.pick_place_step == 3:
            # Close gripper
            self.current_gripper = self.gripper_close
            self.pick_place_step = 4
            self.create_one_shot_timer(0.5)
            
        elif self.pick_place_step == 4:
            # Lift arm
            self.current_lift = self.lift_max
            self.pick_place_step = 5
            self.create_one_shot_timer(0.5)
            
        elif self.pick_place_step == 5:
            # Retract arm
            self.current_extend = self.extend_min
            self.holding_box = True
            self.state = RobotState.NAVIGATE_TO_ZONE
            self.reset_pids()
            self.get_logger().info(f'Picked box, navigating to {self.current_task[1]} zone')

    def navigate_to_zone(self, dt):
        """Navigate to the correct drop zone."""
        if self.current_task is None:
            return

        _, color = self.current_task
        zone_pos = self.drop_zones.get(color, self.home_position)
        target_x, target_y, target_theta = zone_pos
        
        # Calculate distance
        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.position_threshold:
            self.stop_robot()
            self.state = RobotState.PLACE_BOX
            self.pick_place_step = 0
            return
        
        self.navigate_to_point(target_x, target_y, target_theta, dt)

    def execute_place(self):
        """Execute place sequence step by step."""
        if self.pick_place_step == 0:
            # Extend arm
            self.current_extend = self.extend_max
            self.pick_place_step = 1
            self.create_one_shot_timer(0.5)
            
        elif self.pick_place_step == 1:
            # Lower arm
            self.current_lift = self.lift_min
            self.pick_place_step = 2
            self.create_one_shot_timer(0.5)
            
        elif self.pick_place_step == 2:
            # Open gripper
            self.current_gripper = self.gripper_open
            self.pick_place_step = 3
            self.create_one_shot_timer(0.5)
            
        elif self.pick_place_step == 3:
            # Retract arm
            self.current_extend = self.extend_min
            self.pick_place_step = 4
            self.create_one_shot_timer(0.3)
            
        elif self.pick_place_step == 4:
            # Lift arm slightly
            self.current_lift = self.lift_max * 0.5
            self.holding_box = False
            self.get_logger().info(f'Placed {self.current_task[1]} box successfully')
            
            # Check for more tasks
            if self.task_queue:
                self.current_task = self.task_queue.pop(0)
                self.state = RobotState.NAVIGATE_TO_BOX
                self.reset_pids()
            else:
                self.state = RobotState.RETURN_HOME
                self.reset_pids()

    def return_home(self, dt):
        """Return to home position."""
        target_x, target_y, target_theta = self.home_position
        
        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.position_threshold:
            self.stop_robot()
            self.state = RobotState.COMPLETED
            return
        
        self.navigate_to_point(target_x, target_y, target_theta, dt)

    def navigate_to_point(self, target_x, target_y, target_theta, dt, speed_factor=1.0):
        """Navigate to a specific point using PID control."""
        x, y, theta = self.current_pose
        
        # Position errors
        error_x = target_x - x
        error_y = target_y - y
        error_theta = self.normalize_angle(target_theta - theta)
        
        # PID outputs (body frame velocities)
        vx = self.pid_x.compute(error_x, dt) * speed_factor
        vy = self.pid_y.compute(error_y, dt) * speed_factor
        omega = self.pid_theta.compute(error_theta, dt) * speed_factor
        
        # Transform to body frame
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        vx_body = vx * cos_theta + vy * sin_theta
        vy_body = -vx * sin_theta + vy * cos_theta
        
        # Convert to wheel velocities (holonomic kinematics)
        # For 3-wheel omni drive with 120° separation
        wheel_vel = self.body_to_wheel_velocities(vx_body, vy_body, omega)
        
        # Publish
        self.publish_wheel_velocities(wheel_vel)

    def body_to_wheel_velocities(self, vx, vy, omega):
        """
        Convert body velocities to wheel velocities.
        
        Assuming standard 3-wheel omni configuration:
        - Left wheel at 120° (front-left)
        - Right wheel at -120° (front-right)  
        - Back wheel at 180° (rear)
        """
        R = 0.074  # Robot radius (from center to wheel)
        
        # Wheel angles from robot front
        angles = [
            math.radians(120),   # Left
            math.radians(-120),  # Right
            math.radians(180)    # Back
        ]
        
        wheel_vel = []
        for angle in angles:
            # Wheel velocity = projection of linear velocity + rotation component
            v_wheel = (-math.sin(angle) * vx + math.cos(angle) * vy + R * omega)
            wheel_vel.append(v_wheel)
        
        return wheel_vel

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def reset_pids(self):
        """Reset all PID controllers."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_theta.reset()

    def stop_robot(self):
        """Stop all wheel motion."""
        self.publish_wheel_velocities([0.0, 0.0, 0.0])

    def publish_wheel_velocities(self, wheel_vel):
        """Publish wheel velocities."""
        # Clamp velocities
        wheel_vel = [np.clip(v, -self.max_vel, self.max_vel) for v in wheel_vel]
        
        msg = Float64MultiArray()
        msg.data = wheel_vel
        self.wheel_pub.publish(msg)

    def publish_arm_gripper(self):
        """Publish arm and gripper positions."""
        # Arm position
        arm_msg = Float64MultiArray()
        arm_msg.data = [self.current_lift, self.current_extend]
        self.arm_pub.publish(arm_msg)
        
        # Gripper position
        gripper_msg = Float64MultiArray()
        gripper_msg.data = [self.current_gripper, self.current_gripper]
        self.gripper_pub.publish(gripper_msg)

    def create_one_shot_timer(self, duration):
        """Create a one-shot timer for pick/place sequences."""
        if self.pick_place_timer:
            self.pick_place_timer.cancel()
        self.pick_place_timer = self.create_timer(
            duration, 
            self.timer_callback, 
            callback_group=None
        )

    def timer_callback(self):
        """Timer callback for pick/place sequences."""
        if self.pick_place_timer:
            self.pick_place_timer.cancel()
            self.pick_place_timer = None


def main(args=None):
    rclpy.init(args=args)
    controller = Task3Controller()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
