#!/usr/bin/env python3
"""
Arm and Gripper Controller for Task 3 Holonomic Bot.

This node provides control interfaces for the robot arm and gripper system:
- Arm lift control (vertical movement)
- Arm extension control (horizontal reach)
- Gripper open/close control

Publishers:
    /arm_position_controller/commands (Float64MultiArray): Arm joint positions [lift, extend]
    /gripper_position_controller/commands (Float64MultiArray): Gripper positions [left, right]

Subscribers:
    /arm_command (String): High-level arm commands ('lift_up', 'lift_down', 'extend', 'retract')
    /gripper_command (String): High-level gripper commands ('open', 'close', 'half_open')

Services:
    /pick_box (Trigger): Execute pick sequence
    /place_box (Trigger): Execute place sequence
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
import time


class ArmGripperController(Node):
    def __init__(self):
        super().__init__('arm_gripper_controller')

        # ============================================
        # ARM PARAMETERS
        # ============================================
        # Lift joint limits (meters)
        self.lift_min = 0.0
        self.lift_max = 0.08
        self.lift_step = 0.02  # Step size for incremental movement

        # Extension joint limits (meters)
        self.extend_min = 0.0
        self.extend_max = 0.05
        self.extend_step = 0.01

        # ============================================
        # GRIPPER PARAMETERS
        # ============================================
        # Gripper joint limits (meters, negative for closing)
        self.gripper_open = 0.0
        self.gripper_close = -0.015
        self.gripper_half = -0.007

        # ============================================
        # CURRENT STATE
        # ============================================
        self.current_lift = 0.0
        self.current_extend = 0.0
        self.current_gripper_left = 0.0
        self.current_gripper_right = 0.0

        # ============================================
        # PUBLISHERS
        # ============================================
        self.arm_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_position_controller/commands', 
            10
        )
        
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, 
            '/gripper_position_controller/commands', 
            10
        )

        # ============================================
        # SUBSCRIBERS
        # ============================================
        self.arm_cmd_sub = self.create_subscription(
            String,
            '/arm_command',
            self.arm_command_callback,
            10
        )

        self.gripper_cmd_sub = self.create_subscription(
            String,
            '/gripper_command',
            self.gripper_command_callback,
            10
        )

        # ============================================
        # SERVICES
        # ============================================
        self.pick_srv = self.create_service(
            Trigger,
            '/pick_box',
            self.pick_box_callback
        )

        self.place_srv = self.create_service(
            Trigger,
            '/place_box',
            self.place_box_callback
        )

        # ============================================
        # TIMER FOR PUBLISHING
        # ============================================
        self.timer = self.create_timer(0.05, self.publish_commands)

        self.get_logger().info('Arm & Gripper Controller initialized')
        self.get_logger().info(f'Arm lift range: [{self.lift_min}, {self.lift_max}] m')
        self.get_logger().info(f'Arm extend range: [{self.extend_min}, {self.extend_max}] m')

    def arm_command_callback(self, msg):
        """
        Process high-level arm commands.
        
        Commands:
            - 'lift_up': Raise arm by step
            - 'lift_down': Lower arm by step
            - 'lift_max': Raise arm to maximum
            - 'lift_min': Lower arm to minimum
            - 'extend': Extend arm by step
            - 'retract': Retract arm by step
            - 'extend_max': Extend arm to maximum
            - 'retract_full': Retract arm fully
            - 'home': Return to home position
        """
        cmd = msg.data.lower().strip()
        
        if cmd == 'lift_up':
            self.current_lift = min(self.current_lift + self.lift_step, self.lift_max)
            self.get_logger().info(f'Lifting arm to {self.current_lift:.3f}m')
        
        elif cmd == 'lift_down':
            self.current_lift = max(self.current_lift - self.lift_step, self.lift_min)
            self.get_logger().info(f'Lowering arm to {self.current_lift:.3f}m')
        
        elif cmd == 'lift_max':
            self.current_lift = self.lift_max
            self.get_logger().info(f'Arm at maximum height: {self.current_lift:.3f}m')
        
        elif cmd == 'lift_min':
            self.current_lift = self.lift_min
            self.get_logger().info(f'Arm at minimum height: {self.current_lift:.3f}m')
        
        elif cmd == 'extend':
            self.current_extend = min(self.current_extend + self.extend_step, self.extend_max)
            self.get_logger().info(f'Extending arm to {self.current_extend:.3f}m')
        
        elif cmd == 'retract':
            self.current_extend = max(self.current_extend - self.extend_step, self.extend_min)
            self.get_logger().info(f'Retracting arm to {self.current_extend:.3f}m')
        
        elif cmd == 'extend_max':
            self.current_extend = self.extend_max
            self.get_logger().info(f'Arm fully extended: {self.current_extend:.3f}m')
        
        elif cmd == 'retract_full':
            self.current_extend = self.extend_min
            self.get_logger().info('Arm fully retracted')
        
        elif cmd == 'home':
            self.current_lift = self.lift_min
            self.current_extend = self.extend_min
            self.get_logger().info('Arm returned to home position')
        
        else:
            self.get_logger().warn(f'Unknown arm command: {cmd}')

    def gripper_command_callback(self, msg):
        """
        Process high-level gripper commands.
        
        Commands:
            - 'open': Open gripper fully
            - 'close': Close gripper fully
            - 'half': Half-open gripper
        """
        cmd = msg.data.lower().strip()
        
        if cmd == 'open':
            self.current_gripper_left = self.gripper_open
            self.current_gripper_right = self.gripper_open
            self.get_logger().info('Gripper opened')
        
        elif cmd == 'close':
            self.current_gripper_left = self.gripper_close
            self.current_gripper_right = self.gripper_close
            self.get_logger().info('Gripper closed')
        
        elif cmd == 'half':
            self.current_gripper_left = self.gripper_half
            self.current_gripper_right = self.gripper_half
            self.get_logger().info('Gripper half-open')
        
        else:
            self.get_logger().warn(f'Unknown gripper command: {cmd}')

    def pick_box_callback(self, request, response):
        """
        Execute automated pick sequence:
        1. Open gripper
        2. Lower arm
        3. Extend arm
        4. Close gripper
        5. Lift arm
        6. Retract arm
        """
        try:
            self.get_logger().info('Starting pick sequence...')
            
            # Step 1: Open gripper
            self.current_gripper_left = self.gripper_open
            self.current_gripper_right = self.gripper_open
            self.publish_commands()
            time.sleep(0.5)
            
            # Step 2: Lower arm
            self.current_lift = self.lift_min
            self.publish_commands()
            time.sleep(0.5)
            
            # Step 3: Extend arm
            self.current_extend = self.extend_max
            self.publish_commands()
            time.sleep(0.5)
            
            # Step 4: Close gripper
            self.current_gripper_left = self.gripper_close
            self.current_gripper_right = self.gripper_close
            self.publish_commands()
            time.sleep(0.5)
            
            # Step 5: Lift arm
            self.current_lift = self.lift_max
            self.publish_commands()
            time.sleep(0.5)
            
            # Step 6: Retract arm
            self.current_extend = self.extend_min
            self.publish_commands()
            time.sleep(0.3)
            
            response.success = True
            response.message = 'Pick sequence completed successfully'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Pick sequence failed: {str(e)}'
            self.get_logger().error(response.message)
        
        return response

    def place_box_callback(self, request, response):
        """
        Execute automated place sequence:
        1. Extend arm
        2. Lower arm
        3. Open gripper
        4. Retract arm
        5. Lift arm
        """
        try:
            self.get_logger().info('Starting place sequence...')
            
            # Step 1: Extend arm
            self.current_extend = self.extend_max
            self.publish_commands()
            time.sleep(0.5)
            
            # Step 2: Lower arm
            self.current_lift = self.lift_min
            self.publish_commands()
            time.sleep(0.5)
            
            # Step 3: Open gripper
            self.current_gripper_left = self.gripper_open
            self.current_gripper_right = self.gripper_open
            self.publish_commands()
            time.sleep(0.5)
            
            # Step 4: Retract arm
            self.current_extend = self.extend_min
            self.publish_commands()
            time.sleep(0.3)
            
            # Step 5: Lift arm (optional, for clearance)
            self.current_lift = self.lift_max * 0.5
            self.publish_commands()
            time.sleep(0.3)
            
            response.success = True
            response.message = 'Place sequence completed successfully'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Place sequence failed: {str(e)}'
            self.get_logger().error(response.message)
        
        return response

    def publish_commands(self):
        """Publish current arm and gripper positions."""
        # Publish arm position
        arm_msg = Float64MultiArray()
        arm_msg.data = [self.current_lift, self.current_extend]
        self.arm_pub.publish(arm_msg)
        
        # Publish gripper position
        gripper_msg = Float64MultiArray()
        gripper_msg.data = [self.current_gripper_left, self.current_gripper_right]
        self.gripper_pub.publish(gripper_msg)

    def set_arm_position(self, lift, extend):
        """
        Set arm position directly.
        
        Args:
            lift: Lift height in meters (0.0 to 0.08)
            extend: Extension in meters (0.0 to 0.05)
        """
        self.current_lift = max(self.lift_min, min(lift, self.lift_max))
        self.current_extend = max(self.extend_min, min(extend, self.extend_max))

    def set_gripper_position(self, position):
        """
        Set gripper position directly.
        
        Args:
            position: Gripper position in meters (-0.015 to 0.0)
        """
        position = max(self.gripper_close, min(position, self.gripper_open))
        self.current_gripper_left = position
        self.current_gripper_right = position


def main(args=None):
    rclpy.init(args=args)
    controller = ArmGripperController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
