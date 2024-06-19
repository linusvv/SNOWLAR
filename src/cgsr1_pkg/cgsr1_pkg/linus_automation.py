import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')

        # Parameters
        self.down_speed = 0.5  # Speed when moving down
        self.up_speed = -0.5  # Speed when moving up (reverse)
        self.turn_speed = 0.3  # Speed when turning
        self.cross_speed = 0.2  # Speed when moving across the plane
        self.cross_distance = 0.1  # Distance to move across the plane (in meters)

        # State variables
        self.current_y = 0.0
        self.target_y = 0.0
        self.imu_angle = 0.0
        self.stop = False
        self.automation_active = False
        self.state = 'DOWN'

        # Create publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.camera_subscriber = self.create_subscription(Bool, 'camera', self.camera_callback, 10)
        self.y_position_subscriber = self.create_subscription(Float64, 'y_position', self.y_position_callback, 10)
        self.imu_subscriber = self.create_subscription(Float64, 'imu_data', self.imu_callback, 10)
        self.winch_zero_publisher = self.create_publisher(Bool, 'set_winch_zero', 10)
        self.automation_subscriber = self.create_subscription(Bool, 'automation', self.automation_callback, 10)

        # Publish True to /set_winch_zero once
        self.set_winch_zero()

        # Create a timer for control loop
        self.timer = self.create_timer(0.04, self.control_loop)  # Timer set to 25 Hz (0.04 seconds)

    def set_winch_zero(self):
        msg = Bool()
        msg.data = True
        self.winch_zero_publisher.publish(msg)

    def camera_callback(self, msg):
        self.stop = msg.data

    def y_position_callback(self, msg):
        self.current_y = msg.data

    def imu_callback(self, msg):
        self.imu_angle = msg.data

    def automation_callback(self, msg):
        self.automation_active = msg.data
        if not self.automation_active:
            self.publish_velocity(0.0, 0.0)

    def control_loop(self):
        if self.stop or not self.automation_active:
            self.publish_velocity(0.0, 0.0)
            return

        twist = Twist()
        if self.state == 'DOWN':
            twist.linear.x = self.down_speed
            self.publish_velocity_with_correction(twist, 0.0)
            if self.current_y <= self.target_y:
                self.state = 'UP'
                self.target_y = self.current_y + 0.1  # Define your logic for determining the target_y

        elif self.state == 'UP':
            twist.linear.x = self.up_speed
            self.publish_velocity_with_correction(twist, 1.0)
            if self.current_y >= self.target_y:
                self.state = 'TURN_RIGHT'

        elif self.state == 'TURN_RIGHT':
            twist.angular.z = self.turn_speed
            self.publish_velocity_with_correction(twist, 0.5)
            if abs(self.imu_angle - 0.5) < 0.05:  # Allowable error in the angle
                self.state = 'CROSS'
                self.cross_start_y = self.current_y

        elif self.state == 'CROSS':
            twist.linear.x = self.cross_speed
            self.publish_velocity(twist.linear.x, 0.0)
            if abs(self.current_y - self.cross_start_y) >= self.cross_distance:
                self.state = 'TURN_LEFT'

        elif self.state == 'TURN_LEFT':
            twist.angular.z = -self.turn_speed
            self.publish_velocity_with_correction(twist, 0.0)
            if abs(self.imu_angle - 0.0) < 0.05:
                self.state = 'DOWN'
                self.target_y = self.current_y - 0.1  # Define your logic for determining the target_y

    def publish_velocity_with_correction(self, twist, target_angle):
        error = target_angle - self.imu_angle
        if abs(error) > 0.05:  # Allowable error in the angle
            twist.angular.z = self.turn_speed if error > 0 else -self.turn_speed
        else:
            twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

    def publish_velocity(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()
    rclpy.spin(rover_controller)
    rover_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
