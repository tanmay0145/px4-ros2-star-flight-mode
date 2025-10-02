#!/usr/bin/env python3

"""
PX4 Python Star Flight Mode

A clean, production-ready implementation of a custom PX4 flight mode in Python
that makes the drone draw a 5-pointed star pattern in the air.

Usage:
    ros2 run px4_python_star_flight_mode star_flight_mode

Author: Nathaniel Handan <handanfoun@gmail.com>
License: BSD-3-Clause
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleControlMode
)


class FlightState(Enum):
    """Flight state enumeration."""
    INIT = 0
    ARMING = 1
    TAKEOFF = 2
    STAR_PATTERN = 3
    RTL = 4
    COMPLETE = 5


class StarFlightMode(Node):
    """
    PX4 Star Pattern Flight Mode.
    
    Executes a complete autonomous mission:
    1. Arms the vehicle
    2. Takes off to specified altitude
    3. Flies a 5-pointed star pattern
    4. Returns to launch and lands
    """
    
    # Flight parameters
    STAR_RADIUS = 10.0      # meters
    FLIGHT_ALTITUDE = -5.0  # meters (NED frame, negative is up)
    POSITION_TOLERANCE = 1.0 # meters
    
    def __init__(self):
        super().__init__('star_flight_mode')
        
        # QoS profile for PX4 compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Subscribers
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback, qos_profile)
        
        # State variables
        self.state = FlightState.INIT
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_control_mode = VehicleControlMode()
        
        # Mission variables
        self.start_position = [0.0, 0.0, 0.0]
        self.star_waypoints = []
        self.current_waypoint = 0
        self.state_timer = 0
        
        # Data reception flags
        self.data_ready = False
        
        # Main control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Star Flight Mode initialized")
    
    def vehicle_local_position_callback(self, msg):
        """Update vehicle local position."""
        self.vehicle_local_position = msg
        self._check_data_ready()
    
    def vehicle_status_callback(self, msg):
        """Update vehicle status."""
        self.vehicle_status = msg
        self._check_data_ready()
    
    def vehicle_control_mode_callback(self, msg):
        """Update vehicle control mode."""
        self.vehicle_control_mode = msg
    
    def _check_data_ready(self):
        """Check if all required data is available."""
        if not self.data_ready and hasattr(self.vehicle_status, 'arming_state'):
            self.data_ready = True
            self.get_logger().info("PX4 connection established")
    
    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Send a vehicle command."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = float('nan')
        msg.param4 = float('nan')
        msg.param5 = float('nan')
        msg.param6 = float('nan')
        msg.param7 = float('nan')
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
    
    def publish_offboard_control_mode(self):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_trajectory_setpoint(self, x, y, z):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')
        self.trajectory_setpoint_pub.publish(msg)
    
    def calculate_star_waypoints(self):
        """Calculate 5-pointed star waypoints."""
        waypoints = []
        cx, cy = self.start_position[0], self.start_position[1]
        
        # Generate 5 outer points of the star
        for i in range(5):
            angle = i * 72.0 * math.pi / 180.0 - math.pi / 2  # Start pointing North
            x = cx + self.STAR_RADIUS * math.cos(angle)
            y = cy + self.STAR_RADIUS * math.sin(angle)
            waypoints.append([x, y, self.FLIGHT_ALTITUDE])
        
        # Reorder to create star pattern: 0->2->4->1->3
        star_sequence = [0, 2, 4, 1, 3]
        return [waypoints[i] for i in star_sequence]
    
    def distance_to_target(self, target):
        """Calculate distance to target waypoint."""
        dx = self.vehicle_local_position.x - target[0]
        dy = self.vehicle_local_position.y - target[1]
        dz = self.vehicle_local_position.z - target[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def control_loop(self):
        """Main control loop - executed at 10 Hz."""
        self.state_timer += 1
        
        if not self.data_ready:
            return
        
        if self.state == FlightState.INIT:
            # Initialize mission
            self.start_position = [
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            ]
            self.star_waypoints = self.calculate_star_waypoints()
            self.get_logger().info(f"Mission initialized at {self.start_position}")
            self.state = FlightState.ARMING
            self.state_timer = 0
        
        elif self.state == FlightState.ARMING:
            # Send arm command
            if self.state_timer % 20 == 0:  # Every 2 seconds
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
            
            # Check if armed
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Vehicle armed - starting takeoff")
                self.state = FlightState.TAKEOFF
                self.state_timer = 0
            elif self.state_timer > 100:  # 10 seconds timeout
                self.get_logger().error("Arming failed - check pre-flight conditions")
                self.state = FlightState.COMPLETE
        
        elif self.state == FlightState.TAKEOFF:
            # Publish offboard control mode
            self.publish_offboard_control_mode()
            
            # Switch to offboard mode
            if self.state_timer < 20:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            
            # Publish takeoff setpoint
            self.publish_trajectory_setpoint(
                self.start_position[0], 
                self.start_position[1], 
                self.FLIGHT_ALTITUDE
            )
            
            # Check if takeoff complete
            altitude_error = abs(self.vehicle_local_position.z - self.FLIGHT_ALTITUDE)
            if altitude_error < 1.0 and self.state_timer > 50:  # 5 seconds minimum
                self.get_logger().info("Takeoff complete - starting star pattern")
                self.current_waypoint = 0
                self.state = FlightState.STAR_PATTERN
                self.state_timer = 0
        
        elif self.state == FlightState.STAR_PATTERN:
            # Publish offboard control mode
            self.publish_offboard_control_mode()
            
            # Navigate to current waypoint
            if self.current_waypoint < len(self.star_waypoints):
                target = self.star_waypoints[self.current_waypoint]
                self.publish_trajectory_setpoint(target[0], target[1], target[2])
                
                # Check if waypoint reached
                if self.distance_to_target(target) < self.POSITION_TOLERANCE:
                    self.current_waypoint += 1
                    self.get_logger().info(f"Waypoint {self.current_waypoint}/{len(self.star_waypoints)} reached")
            else:
                # Star pattern complete
                self.get_logger().info("Star pattern complete - returning to launch")
                self.state = FlightState.RTL
                self.state_timer = 0
        
        elif self.state == FlightState.RTL:
            # Send RTL command
            if self.state_timer == 10:  # Send once after 1 second
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            
            # Check for landing
            if abs(self.vehicle_local_position.z) < 1.0 and self.state_timer > 100:
                self.get_logger().info("Landing complete")
                self.state = FlightState.COMPLETE
        
        elif self.state == FlightState.COMPLETE:
            # Disarm vehicle
            if self.state_timer == 10:  # Send once after 1 second
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self.get_logger().info("Mission completed successfully")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = StarFlightMode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()