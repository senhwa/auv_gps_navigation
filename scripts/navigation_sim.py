#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Twist
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
from pyproj import Proj, Transformer, CRS
import numpy as np
import tf_transformations

class AUVSimulationNode(Node):
    def __init__(self):
        super().__init__('auv_simulation_node')

        # Parameters
        self.mass = 50.0  # AUV mass in kg
        self.drag_coefficient_x = 50.0  # Hydrodynamic drag coefficient in x-axis
        self.drag_coefficient_y = 50.0  # Hydrodynamic drag coefficient in y-axis
        self.drag_coefficient_z = 50.0  # Hydrodynamic drag coefficient in z-axis
        self.buoyancy = 510.0  # Buoyancy in Newton
        self.inertia_z = 200.0  # Rotational inertia around z-axis
        self.angular_drag_coefficient = 20.0  # Angular resistance coefficient  # Angular resistance coefficient

        # Initial states
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.fx = 0.0
        self.fy = 0.0
        self.fz = 0.0
        self.torque_z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        # Coordinate transformer
        self.proj_wgs84 = CRS("WGS84")
        self.proj_target = CRS("EPSG:6676")
        self.proj_transformer = Transformer.from_crs(self.proj_target,self.proj_wgs84)

        # Subscribers
        self.subscription_wrench = self.create_subscription(Wrench, '/auv/wrench', self.wrench_callback, 10)

        # Publishers
        self.gps_publisher = self.create_publisher(NavSatFix, '/sensing/gps/data', 10)
        self.depth_publisher = self.create_publisher(Float32, '/sensing/depth/data', 10)
        self.imu_publisher = self.create_publisher(Imu, '/sensing/imu/data', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/simulation/velocity', 10)

        # Timer for updating the simulation
        self.create_timer(0.1, self.update_simulation)  # 10 Hz update rate

    def wrench_callback(self, msg: Wrench):
        # Extract forces and torques from the wrench message
        self.fx = msg.force.x
        self.fy = msg.force.y
        self.fz = msg.force.z
        self.torque_z = msg.torque.z

    def update_simulation(self):
        # Time step
        dt = 0.1  # 10 Hz

        # Update linear accelerations
        ax = (self.fx - self.drag_coefficient_x * self.vx) / self.mass
        ay = (self.fy - self.drag_coefficient_y * self.vy) / self.mass
        az = (self.fz - self.buoyancy + self.mass * 9.81 - self.drag_coefficient_z * self.vz) / self.mass
        if self.z < 0:
            az = 0.1
        # Update angular acceleration
        angular_acc = (self.torque_z - self.angular_drag_coefficient * self.yaw_rate) / self.inertia_z

        # Update velocities
        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt
        self.yaw_rate += angular_acc * dt

        # Update positions considering the current yaw angle
        self.x += (self.vx * np.cos(self.yaw) - self.vy * np.sin(self.yaw)) * dt
        self.y += (self.vx * np.sin(self.yaw) + self.vy * np.cos(self.yaw)) * dt
        self.z += self.vz * dt
        self.yaw += self.yaw_rate * dt

        # Publish updated GPS position
        lon, lat = self.proj_transformer.transform(self.x, self.y)
        gps_msg = NavSatFix()
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = -self.z
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        self.gps_publisher.publish(gps_msg)

        # Publish updated depth with stamped message
        depth_msg = Float32()
        depth_msg.data = self.z
        self.depth_publisher.publish(depth_msg)

        # Publish updated IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.angular_velocity.z = self.yaw_rate
        imu_msg.orientation = self.get_orientation_quaternion()
        self.imu_publisher.publish(imu_msg)

        # Publish velocity and angular velocity
        velocity_msg = Twist()
        velocity_msg.linear.x = self.vx
        velocity_msg.linear.y = self.vy
        velocity_msg.linear.z = self.vz
        velocity_msg.angular.z = self.yaw_rate
        self.velocity_publisher.publish(velocity_msg)

    def get_orientation_quaternion(self):
        # Calculate quaternion from yaw angle
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        imu_orientation = Imu().orientation
        imu_orientation.x = q[0]
        imu_orientation.y = q[1]
        imu_orientation.z = q[2]
        imu_orientation.w = q[3]
        return imu_orientation


def main(args=None):
    rclpy.init(args=args)
    node = AUVSimulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()