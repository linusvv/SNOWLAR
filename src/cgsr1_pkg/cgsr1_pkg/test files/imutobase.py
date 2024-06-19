import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import math

class JoyToCmdVelAndServo(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel_and_servo')
        self.subscription = self.create_subscription(
            Imu,
            '/olive/imu/imu/filtered_ahrs',
            self.imu_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.servo_publisher = self.create_publisher(Float32, '/olive/servo/hook/goal/position', 10)
        
        self.max_velocity = 18.0  # Maximum velocity
        self.servo_position = 0.0  # Initial servo position
        self.angle_threshold = 45  # Angle threshold for stopping the robot
        self.tilted = False  # Flag to indicate if the device is tilted

    def imu_callback(self, msg):
        # Extract orientation data
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Calculate pitch and roll from quaternion
        pitch = math.asin(2.0 * (w * y - z * x))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

        # Check if the robot is tilted more than 45 degrees
        if abs(roll) > math.radians(self.angle_threshold) or abs(pitch) > math.radians(self.angle_threshold):
            self.tilted = True
        else:
            self.tilted = False

        # Publish angle or zeros based on tilt status
        if self.tilted:
            twist = Twist()
            twist.linear.x = 0.0  # Explicitly set to float
            twist.linear.y = 0.0  # Explicitly set to float
            self.cmd_vel_publisher.publish(twist)
            self.servo_publisher.publish(Float32(data=0.0))  # Publish zero angle
        else:
            twist = Twist()
            twist.linear.x = y * self.max_velocity  # Mapping y-Axis to linear velocity
            twist.linear.y = x * self.max_velocity  # Mapping x-Axis to angular velocity
            self.cmd_vel_publisher.publish(twist)
            self.servo_publisher.publish(Float32(data=self.servo_position))  # Publish the angle

def main(args=None):
    rclpy.init(args=args)
    joy_to_cmd_vel_and_servo = JoyToCmdVelAndServo()
    rclpy.spin(joy_to_cmd_vel_and_servo)
    joy_to_cmd_vel_and_servo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

