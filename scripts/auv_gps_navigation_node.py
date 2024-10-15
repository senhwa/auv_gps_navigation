#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, Wrench
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
import tf_transformations
from pyproj import Proj, transform
from geometry_msgs.msg import TransformStamped
import math
import time
from simple_pid import PID

class AUVNavigationNode(Node):
    def __init__(self):
        super().__init__('auv_navigation_node')

        # Declare ROS parameters for various thresholds and limits
        self.declare_parameter('waypoint_threshold', 1.0)  # Meters
        self.declare_parameter('angle_threshold', 10.0)  # Degrees
        self.declare_parameter('gps_timeout', 5.0)  # Seconds
        self.declare_parameter('x_output_limit', 1.0)  # Maximum limit for x control effort

        # Load parameters with default values
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').get_parameter_value().double_value
        self.angle_threshold = math.radians(self.get_parameter('angle_threshold').get_parameter_value().double_value)
        self.gps_timeout = self.get_parameter('gps_timeout').get_parameter_value().double_value
        self.x_output_limit = self.get_parameter('x_output_limit').get_parameter_value().double_value

        # Subscribers for GPS, IMU, and depth sensors
        self.subscription_gps = self.create_subscription(NavSatFix, '/sensing/gps/data', self.gps_callback, 10)
        self.subscription_imu = self.create_subscription(Imu, '/sensing/imu/data', self.imu_callback, 10)
        self.subscription_depth = self.create_subscription(Float32, '/sensing/depth/data', self.depth_callback, 10)

        # Publishers for AUV pose and wrench commands
        self.pose_publisher = self.create_publisher(PoseStamped, '/auv/pose', 10)
        self.wrench_publisher = self.create_publisher(Wrench, '/auv/wrench', 10)

        # Transform broadcaster for publishing AUV pose as a TF transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # Coordinate transformation using pyproj (WGS84 to target EPSG:6676)
        self.proj_wgs84 = Proj('epsg:4326')
        self.proj_target = Proj('epsg:6676')

        # Variables for storing current sensor data and state
        self.current_lat = None
        self.current_lon = None
        self.current_depth = None
        self.current_orientation = None
        self.waypoints = [
            (37.7749, -122.4194),  # Example waypoint 1
            (37.7750, -122.4180),  # Example waypoint 2
            (37.7751, -122.4170)   # Example waypoint 3
        ]
        self.current_waypoint_index = 0
        self.last_gps_time = time.time()
        self.waypoint_start_time = time.time()
        self.gps_lost = False

        # PID Controllers for x propulsion and yaw control
        self.pid_x = PID(1.0, 0.1, 0.05, setpoint=0)
        self.pid_x.output_limits = (-self.x_output_limit, self.x_output_limit)  # Limit x control effort
        self.pid_yaw = PID(1.0, 0.1, 0.05, setpoint=0)
        self.pid_yaw.output_limits = (-1.0, 1.0)  # Limit yaw control effort

        # Timer for logging status information
        self.create_timer(2.0, self.log_status_callback)  # Log status every 2 seconds

    def gps_callback(self, msg):
        # Update current GPS coordinates and reset GPS timeout
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.last_gps_time = time.time()

        # Log GPS signal restoration if it was previously lost
        if self.gps_lost:
            self.get_logger().info('GPS signal restored. Resuming operations.')
            self.gps_lost = False

    def imu_callback(self, msg):
        # Update current orientation from IMU data
        self.current_orientation = msg.orientation

    def depth_callback(self, msg):
        # Update current depth from depth sensor data
        self.current_depth = msg.data

    def publish_pose(self):
        # Ensure all necessary data is available before publishing pose
        if self.current_lat is None or self.current_lon is None or self.current_depth is None or self.current_orientation is None:
            return

        # Convert GPS coordinates to target UTM coordinate system (EPSG:6676)
        x, y = transform(self.proj_wgs84, self.proj_target, self.current_lon, self.current_lat)
        z = self.current_depth

        # Create PoseStamped message for AUV position and orientation
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation = self.current_orientation

        # Publish pose to '/auv/pose'
        self.pose_publisher.publish(pose_msg)

        # Publish transform for TF2 broadcasting
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = self.current_orientation

        self.tf_broadcaster.sendTransform(t)

    def publish_wrench(self):
        # Check if GPS data is outdated (timeout)
        if time.time() - self.last_gps_time > self.gps_timeout:
            if not self.gps_lost:
                self.get_logger().warn('GPS signal lost. Stopping AUV.')
                self.gps_lost = True
            self.stop_auv()  # Stop AUV if GPS signal is lost
            return

        # Ensure all necessary data is available before calculating wrench
        if self.current_lat is None or self.current_lon is None or self.current_orientation is None:
            return

        # Get current target waypoint
        target_lat, target_lon = self.waypoints[self.current_waypoint_index]

        # Convert target waypoint GPS to UTM coordinates
        target_x, target_y = transform(self.proj_wgs84, self.proj_target, target_lon, target_lat)
        current_x, current_y = transform(self.proj_wgs84, self.proj_target, self.current_lon, self.current_lat)

        # Calculate relative position to target waypoint
        delta_x = target_x - current_x
        delta_y = target_y - current_y
        target_angle = math.atan2(delta_y, delta_x)  # Calculate angle to the target waypoint

        # Extract current yaw from orientation quaternion
        _, _, current_yaw = tf_transformations.euler_from_quaternion([
            self.current_orientation.x,
            self.current_orientation.y,
            self.current_orientation.z,
            self.current_orientation.w
        ])

        # Calculate difference between target angle and current yaw
        angle_diff = self.calculate_angle_difference(target_angle, current_yaw)

        # Check if the AUV has reached the waypoint within the specified threshold
        if self.is_waypoint_reached(delta_x, delta_y):
            self.move_to_next_waypoint()  # Move to the next waypoint
            return

        # Create Wrench message using PID controllers for x propulsion and yaw control
        wrench_msg = self.create_pid_wrench_message(delta_x, angle_diff)

        # Publish wrench to '/auv/wrench'
        self.wrench_publisher.publish(wrench_msg)

    def calculate_angle_difference(self, target_angle, current_yaw):
        # Calculate difference between target angle and current yaw, normalize to [-pi, pi]
        angle_diff = target_angle - current_yaw
        return (angle_diff + math.pi) % (2 * math.pi) - math.pi

    def is_waypoint_reached(self, delta_x, delta_y):
        # Calculate distance to the waypoint
        distance_to_waypoint = math.sqrt(delta_x ** 2 + delta_y ** 2)
        # Return True if distance is within the threshold
        return distance_to_waypoint < self.waypoint_threshold

    def move_to_next_waypoint(self):
        # Move to the next waypoint in the list
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
        self.waypoint_start_time = time.time()
        self.get_logger().info(f'Moving to next waypoint: Waypoint {self.current_waypoint_index + 1}')

    def create_pid_wrench_message(self, delta_x, angle_diff):
        # Create Wrench message to control AUV
        wrench_msg = Wrench()
        # Calculate yaw control effort using PID controller
        yaw_control_effort = self.pid_yaw(angle_diff)
        wrench_msg.torque.z = yaw_control_effort

        # Apply force in x direction using PID for x propulsion
        if abs(angle_diff) <= self.angle_threshold:
            x_control_effort = self.pid_x(delta_x)
            wrench_msg.force.x = x_control_effort
        else:
            # If the angle difference is too large, stop forward motion
            wrench_msg.force.x = 0.0

        return wrench_msg

    def stop_auv(self):
        # Publish zero wrench to stop AUV motion
        wrench_msg = Wrench()
        wrench_msg.force.x = 0.0
        wrench_msg.force.y = 0.0
        wrench_msg.force.z = 0.0
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = 0.0
        self.wrench_publisher.publish(wrench_msg)

    def log_status_callback(self):
        # Log current status of the AUV every 2 seconds
        if self.current_lat is not None and self.current_lon is not None:
            target_lat, target_lon = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(f'Current Position: Lat {self.current_lat}, Lon {self.current_lon}')
            self.get_logger().info(f'Target Waypoint {self.current_waypoint_index + 1}: Lat {target_lat}, Lon {target_lon}')
            self.get_logger().info(f'Time Tracking Current Waypoint: {time.time() - self.waypoint_start_time:.2f} seconds')

    def timer_callback(self):
        # Periodically publish pose and wrench commands
        self.publish_pose()
        self.publish_wrench()


def main(args=None):
    rclpy.init(args=args)
    node = AUVNavigationNode()
    timer_period = 0.1  # 10 Hz
    node.create_timer(timer_period, node.timer_callback)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()