import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading

class EncoderToCmdVel(Node):
    def __init__(self):
        super().__init__('encoder_to_cmd_vel')

        # Initialize serial port
        self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        time.sleep(2)

        # Initialize cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Start thread to read data from Arduino
        threading.Thread(target=self.read_from_arduino, daemon=True).start()

    def read_from_arduino(self):
        while True:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received: {line}")

                # Parse encoder data
                try:
                    # Extract wheel velocities from the received string
                    wheel_data = {item[0]: float(item[1:]) for item in line.split(',')}
                    left_velocity = (wheel_data['A'] + wheel_data['B']) / 2
                    right_velocity = (wheel_data['C'] + wheel_data['D']) / 2

                    # Compute linear and angular velocities
                    self.process_wheel_data(left_velocity, right_velocity)
                except (ValueError, KeyError):
                    self.get_logger().warn("Invalid encoder data format!")

    def process_wheel_data(self, left_velocity, right_velocity):
        # Parameters (adjust based on your robot's specifications)
        wheel_base = 0.3  # Distance between left and right wheels in meters

        # Compute linear and angular velocities
        linear_velocity = (left_velocity + right_velocity) / 2
        angular_velocity = (right_velocity - left_velocity) / wheel_base

        # Publish cmd_vel
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)

        self.get_logger().info(f"Published cmd_vel: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToCmdVel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exiting program.")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
