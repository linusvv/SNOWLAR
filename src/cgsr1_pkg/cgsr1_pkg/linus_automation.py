import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, Int32, Float32

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')

        # Parameters
        self.down_speed = 1.0  # Speed when moving down
        self.up_speed = -1.0  # Speed when moving up (reverse)
        self.turn_speed = 1.0  # Speed when turning
        self.cross_speed = 1.0  # Speed when moving across the plane
        self.cross_distance = 1.0  # Distance to move across the plane (in meters)

        # State variables
        self.current_y = 0.0
        self.target_y = 1.3
        self.imu_angle = 0.0
        self.stop = False
        self.automation_active = False
        self.state = 'DOWN'
        self.cross_timer = None

        self.apriltag_left = 134
        self.apriltag_right = 132

        # Create publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'automation', 10)
        self.camera_subscriber = self.create_subscription(Int32, 'apriltag_id', self.camera_callback, 10)
        self.y_position_subscriber = self.create_subscription(Twist, 'motor_turns', self.motor_turns_callback, 10)
        self.imu_subscriber = self.create_subscription(Float32, 'imu_data', self.imu_callback, 10)
        self.winch_zero_publisher = self.create_publisher(Bool, 'set_winch_zero', 10)
        self.automation_subscriber = self.create_subscription(Bool, 'autonomous', self.automation_callback, 10)

        # Create a timer for control loop
        self.timer = self.create_timer(0.04, self.control_loop)  # Timer set to 25 Hz (0.04 seconds)

    def set_winch_zero(self):
        if self.automation_active:
            msg = Bool()
            msg.data = True
            self.winch_zero_publisher.publish(msg)

    def camera_callback(self, msg):
        if self.automation_active:
            tag_id = msg.data
            if tag_id == self.apriltag_left:
                self.state = 'APRILTAG_LEFT'
                self.get_logger().info('Apriltag left detected')
            elif tag_id == self.apriltag_right:
                self.state = 'APRILTAG_RIGHT'
                self.get_logger().info('Apriltag right detected')

    def motor_turns_callback(self, msg):
        self.current_y = msg.linear.z

    def imu_callback(self, msg):
        self.imu_angle = msg.data

    def automation_callback(self, msg):
        self.automation_active = msg.data
        if not self.automation_active:
            self.publish_turn(0.0)
        else:
            self.set_winch_zero()
            self.get_logger().info('Set winch position to zero')

    def control_loop(self):
        if not self.automation_active:
            self.publish_turn(0.0)
            return

        twist = Twist()
        if self.state == 'DOWN':
            self.get_logger().info('Driving down')
            self.publish_straigth(self.down_speed, 0.0)
            if self.current_y >= self.target_y:
                self.target_y = 0.0 
                self.state = 'UP'
                

        elif self.state == 'UP':
            self.get_logger().info('Driving up')
            twist.linear.y = self.up_speed
            self.publish_straigth(self.up_speed, 0.0)
            if self.current_y <= self.target_y:
                self.state = 'TURN_RIGHT'

        elif self.state == 'TURN_RIGHT':
            self.get_logger().info('Turning right until facing across')
            self.publish_turn(self.turn_speed)
            if abs(self.imu_angle - (- 0.5)) < 0.05:  # Allowable error in the angle
                self.state = 'CROSS'
                self.cross_timer = self.create_timer(3.0, self.cross_timer_callback)  # Start a 3-second timer

        elif self.state == 'CROSS':
            self.get_logger().info('Moving across in one direction')
            self.publish_straigth(self.cross_speed, -0.5)

        elif self.state == 'TURN_LEFT':
            self.get_logger().info('Turning left until facing down')
            self.publish_turn(-self.turn_speed)
            if self.imu_angle  > 0.02:
                self.state = 'DOWN'
                self.get_logger().info('%d' % self.current_y)
                self.target_y = 1.3

        elif self.state == 'APRILTAG_LEFT':
            self.get_logger().info('State apriltag reached')
            self.publish_turn(-self.turn_speed)
            if self.imu_angle > 0.02:
                self.state = 'DOWN_AGAIN'
                self.target_y = 1.3

        elif self.state == 'DOWN_AGAIN':
            self.get_logger().info('Last time down')
            self.publish_straigth(self.down_speed, 0.0)
            if self.current_y >= self.target_y:
                self.state = 'UP_AGAIN'
                self.target_y = 0.0

        elif self.state == 'UP_AGAIN':
            self.get_logger().info('Last time up')
            self.publish_straigth(self.up_speed, 0.0)
            if self.current_y < 0.0:
                self.state = 'TURN_LEFT_AGAIN'

        elif self.state == 'TURN_LEFT_AGAIN':
            self.get_logger().info('Turning left until facing across')
            self.publish_turn(-self.turn_speed)
            if abs(self.imu_angle - (0.5)) < 0.05:
                self.get_logger().info('%d' % self.current_y)
                self.state == 'CROSS_BACK'
                self.cross_timer = self.create_timer(20.0, self.cross_timer_callback)  # Start a 3-second timer


        elif self.state == 'CROSS_BACK':
            self.get_logger().info('Driving back across')
            self.publish_straigth(self.cross_speed, 0.5)
            # Assuming you want to cross back to the original y position

        elif self.state == 'APRILTAG_RIGHT':
            self.get_logger().info('Turning right until facing home')
            self.publish_turn(-self.turn_speed)
            if self.imu_angle  > 0.02:
                self.state = 'STOP'


        elif self.state == 'STOP':
            self.publish_turn(0.0, 0.0)
            self.automation_active = False
            # Wait until new /autonomous true is sent

    def cross_timer_callback(self):
        self.cross_timer.cancel()  # Cancel the timer
        self.state = 'TURN_LEFT'

    def cross_back_timer_callback(self):
        self.cross_back_timer.cancel()  # Cancel the timer
        self.state = 'STOP'

    def publish_straigth(self, linear_y, target_angle):
        twist = Twist()
        if not self.automation_active:
            return
        error = target_angle - self.imu_angle
        if abs(error) > 0.1:  # Allowable error in the angle
            twist.linear.x = -self.turn_speed if error < 0.0 else self.turn_speed
        else:
            twist.linear.x = 0.0
        twist.linear.y = linear_y
        self.cmd_vel_publisher.publish(twist)

    def publish_turn(self, linear_x):
        if not self.automation_active:
            return
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = 0.0
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()
    rclpy.spin(rover_controller)
    rover_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
