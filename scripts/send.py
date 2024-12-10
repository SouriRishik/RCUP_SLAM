import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

# Initialize the serial connection to Arduino
arduino_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Wait for the connection to establish

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')
        
        # Initial PWM values
        self.left_pwm = 0
        self.right_pwm = 0

        # Create a subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        # Linear and angular velocities from Twist message
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Base and max PWM values
        max_pwm = 255
        min_pwm = -255  # Minimum PWM for reverse movement
        base_pwm = 50  # Base PWM for normal movement

        # Define PWM values for each control key
        if linear_vel > 0 and angular_vel == 0:         # `i` - Forward straight
            self.left_pwm = self.right_pwm = base_pwm + int(linear_vel * 100)
        
        elif linear_vel == 0 and angular_vel < 0:       # `j` - Left turn (sharp)
            self.left_pwm = -base_pwm + int(abs(angular_vel) * -100)
            self.right_pwm = base_pwm - int(abs(angular_vel) * 100)

        elif linear_vel == 0 and angular_vel > 0:       # `l` - Right turn (sharp)
            self.left_pwm = base_pwm - int(abs(angular_vel) * 100)
            self.right_pwm = -base_pwm + int(abs(angular_vel) * -100)

        elif linear_vel > 0 and angular_vel < 0:        # `u` - Left radius turn (increase right speed)
            self.left_pwm = base_pwm + int(linear_vel * 100)
            self.right_pwm = base_pwm + int(linear_vel * 20)

        elif linear_vel > 0 and angular_vel > 0:        # `o` - Right radius turn (increase left speed)
            self.left_pwm = base_pwm + int(linear_vel * 20)
            self.right_pwm = base_pwm + int(linear_vel * 100)

        elif linear_vel == 0 and angular_vel == 0:      # `k` - Stop (explicit stop)
            self.left_pwm = self.right_pwm = 0

        elif linear_vel < 0 and angular_vel < 0:        # `m` - Reverse left radius turn (decrease right speed)
            self.left_pwm = -base_pwm - int(abs(linear_vel) * -20)
            self.right_pwm = -base_pwm - int(abs(linear_vel) * -100)

        elif linear_vel < 0 and angular_vel > 0:        # `.` - Reverse right radius turn (decrease left speed)
            self.left_pwm = -base_pwm + int(abs(linear_vel) * -100)
            self.right_pwm = -base_pwm + int(abs(linear_vel) * -20)

        elif linear_vel < 0 and angular_vel == 0:       # `,` - Reverse straight
            # Set reverse straight PWM values (ensure they are negative)
            self.left_pwm = self.right_pwm = -base_pwm - int(linear_vel * -100)
        
        # If linear_vel is negative (for reverse), clamp values between -255 and 0
        if linear_vel < 0:  # Reverse movement
            self.left_pwm = max(min_pwm, self.left_pwm)
            self.right_pwm = max(min_pwm, self.right_pwm)

        # Clamp PWM values for forward motion (0 to max_pwm)
        elif linear_vel >= 0:  # Forward motion
            self.left_pwm = max(0, min(self.left_pwm, max_pwm))
            self.right_pwm = max(0, min(self.right_pwm, max_pwm))

        # Send the calculated PWM to the Arduino
        self.send_pwm_to_arduino(self.left_pwm, self.right_pwm)

    def send_pwm_to_arduino(self, left, right):
        command = f"{left},{right}\n"
        arduino_port.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent PWM: Left={left}, Right={right}")

    def destroy_node(self):
        arduino_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_control = MotorControl()

    try:
        rclpy.spin(motor_control)
    except KeyboardInterrupt:
        print("Exiting motor control.")
    finally:
        motor_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
