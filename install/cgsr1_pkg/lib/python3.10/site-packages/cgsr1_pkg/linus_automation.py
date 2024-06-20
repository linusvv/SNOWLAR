import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, Int32, Float32
from rclpy.timer import Timer

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
        self.target_y = 0.7
        self.imu_angle = 0.0
        self.stop = False
        self.automation_active = False
        self.state = 'DOWN'
        self.cross_timer = None

        self.apriltag_left = 132
        self.apriltag_right = 134

        # Create publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.camera_subscriber = self.create_subscription(Int32, 'apriltag_id', self.camera_callback, 10)
        self.y_position_subscriber = self.create_subscription(Twist, 'motor_turns', self.motor_turns_callback, 10)
        self.imu_subscriber = self.create_subscription(Float32, 'imu_data', self.imu_callback, 10)
        self.winch_zero_publisher = self.create_publisher(Bool, 'set_winch_zero', 10)
        self.automation_subscriber = self.create_subscription(Bool, 'autonomous', self.automation_callback, 10)

        # Create a timer for control loop
        self.timer = self.create_timer(0.04, self.control_loop)  # Timer set to 25 Hz (0.04 seconds)

    def set_winch_zero(self):
        msg = Bool()
        msg.data = True
        self.winch_zero_publisher.publish(msg)

    def camera_callback(self, msg):
        tag_id = msg.data
        if tag_id == self.apriltag_left:
            self.state = 'APRILTAG_LEFT'
            print('Apriltag left detected')
        elif tag_id == self.apriltag_right:
            self.state = 'APRILTAG_RIGHT'
            print('Apriltag right detected')

    def motor_turns_callback(self, msg):
        self.current_y = msg.linear.z

    def imu_callback(self, msg):
        self.imu_angle = msg.data

    def automation_callback(self, msg):
        self.automation_active = msg.data
        if not self.automation_active:
            self.publish_velocity(0.0, 0.0)
        else:
            # Publish True to /set_winch_zero once
            self.set_winch_zero()
            print('Set winch position to zero (Hopefully??)')

    def control_loop(self):
        if not self.automation_active:
            self.publish_velocity(0.0, 0.0)
            return

        twist = Twist()
        if self.state == 'DOWN':
            print('Driving down')
            twist.linear.y = self.down_speed
            self.publish_velocity_with_correction(twist, 0.0)
            if self.current_y >= self.target_y:
                self.state = 'UP'
                self.target_y = 0.1 

        elif self.state == 'UP':
            print('Driving up')
            twist.linear.y = self.up_speed
            self.publish_velocity_with_correction(twist, 0.0)
            if self.current_y <= 0.05:
                self.state = 'TURN_RIGHT'

        elif self.state == 'TURN_RIGHT':
            print('Turning right until facing across')
            twist.linear.y = self.turn_speed
            if abs(self.imu_angle - 0.5) < 0.05:  # Allowable error in the angle
                self.state = 'CROSS'
                self.cross_start_y = self.current_y
                self.cross_timer = self.create_timer(3.0, self.cross_timer_callback)  # Start a 3-second timer

        elif self.state == 'CROSS':
            print('Moving across in one direction')
            twist.linear.x = self.cross_speed
            self.publish_velocity_with_correction(twist, 0.5)
            self.publish_velocity(twist.linear.x, 0.0)

        elif self.state == 'TURN_LEFT':
            print('Turning right until facing down')
            twist.angular.z = -self.turn_speed
            self.publish_velocity_with_correction(twist, 0.0)
            if abs(self.imu_angle - 0.0) < 0.05:
                self.state = 'DOWN'
                self.target_y = self.current_y - 0.1  # Define your logic for determining the target_y

        elif self.state == 'APRILTAG_LEFT':
            print('State apriltag reached')
            twist.angular.z = -self.turn_speed
            self.publish_velocity_with_correction(twist, -0.5)
            if abs(self.imu_angle + 0.5) < 0.05:
                self.state = 'DOWN_AGAIN'
                self.target_y = self.current_y + 0.1

        elif self.state == 'DOWN_AGAIN':
            print('Last time down')
            twist.linear.y = self.down_speed
            self.publish_velocity_with_correction(twist, 0.0)
            if self.current_y >= self.target_y:
                self.state = 'UP_AGAIN'
                self.target_y = 0.1

        elif self.state == 'UP_AGAIN':
            print('Last time up')
            twist.linear.y = self.up_speed
            self.publish_velocity_with_correction(twist, 0.0)
            if self.current_y < 0:
                self.state = 'TURN_LEFT_AGAIN'

        elif self.state == 'TURN_LEFT_AGAIN':
            print('Turn left until facing across')
            twist.angular.z = -self.turn_speed
            self.publish_velocity_with_correction(twist, 0.0)

        elif self.state == 'CROSS_BACK':
            print('Driving back across')
            twist.linear.x = -self.cross_speed
            self.publish_velocity(twist.linear.x, 0.0)
            # Assuming you want to cross back to the original y position

        elif self.state == 'APRILTAG_RIGHT':
            print('Driving until ')
            self.publish_velocity(0.0, 0.0)

        elif self.state == 'TURN_RIGHT_STOP':
            twist.angular.z = self.turn_speed
            self.publish_velocity_with_correction(twist, 0.5)
            if abs(self.imu_angle - 0.5) < 0.05:
                self.state = 'STOP'

        elif self.state == 'STOP':
            self.publish_velocity(0.0, 0.0)
            # Wait until new /autonomous true is sent

    def cross_timer_callback(self):
        self.cross_timer.cancel()  # Cancel the timer
        self.state = 'TURN_LEFT'

    def publish_velocity_with_correction(self, twist, target_angle):
        error = target_angle - self.imu_angle
        if abs(error) > 0.05:  # Allowable error in the angle
            twist.linear.y = self.turn_speed if error > 0 else -self.turn_speed
        else:
            twist.linear.y = 0.0

        self.cmd_vel_publisher.publish(twist)

    def publish_velocity(self, linear_x, linear_y):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()
    rclpy.spin(rover_controller)
    rover_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
