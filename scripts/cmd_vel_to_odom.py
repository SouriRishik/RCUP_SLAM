#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import math
import time

class CmdVelToOdom(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_odom')

        # Publisher for /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Subscribe to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Initial position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation in radians

        # Initialize previous time
        self.last_time = time.time()

    def cmd_vel_callback(self, msg):
        # Get the current time and compute time elapsed
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Extract linear and angular velocities from the message
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Compute the change in position
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        delta_theta = angular_velocity * dt

        # Update position
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to the range [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Populate odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # Orientation as a quaternion
        odom.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.theta / 2),
            w=math.cos(self.theta / 2)
        )

        # Velocity
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        # Publish the odometry message
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
