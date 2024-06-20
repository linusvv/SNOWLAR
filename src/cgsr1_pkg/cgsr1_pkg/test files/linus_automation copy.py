import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Float32
import threading

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')

        # Parameters
        self.down_speed = 0.5  # Speed when moving down
        self.up_speed = -0.5  # Speed when moving up (reverse)
        self.turn_speed = 0.3  # Speed when turning
        self.cross_speed = 0.2  # Speed when moving across the plane

        # State variables
        self.current_y = 0.0
        self.imu_angle = 0.0
        self.automation_active = False

        self.can_log_apriltag = True
        self.apriltag_left = False
        self.apriltag_right = False

        # Create publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'automation', 10)
        self.camera_subscriber = self.create_subscription(Int32, 'apriltag_id', self.camera_callback, 10)
        self.y_position_subscriber = self.create_subscription(Twist, 'motor_turns', self.motor_turns_callback, 10)
        self.imu_subscriber = self.create_subscription(Float32, 'imu_data', self.imu_callback, 10)
        self.automation_subscriber = self.create_subscription(Bool, 'autonomous', self.automation_callback, 10)

        # Threading setup
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def camera_callback(self, msg):
        with self.lock:
            self.get_logger().info(f'Camera callback with tag id: {msg.data}')
            if self.can_log_apriltag:
                tag_id = msg.data
                if tag_id == 132:  # Example apriltag ID for left
                    self.get_logger().info('Apriltag left detected')
                    self.apriltag_left = True
                elif tag_id == 134:  # Example apriltag ID for right
                    self.get_logger().info('Apriltag right detected')
                    self.apriltag_right = True

                self.can_log_apriltag = False
                self.create_timer(5.0, self.reset_apriltag_logging)
                self.condition.notify_all()

    def reset_apriltag_logging(self):
        with self.lock:
            self.can_log_apriltag = True
            self.get_logger().info('Apriltag logging reset')

    def motor_turns_callback(self, msg):
        with self.lock:
            self.current_y = msg.linear.z
            self.get_logger().info(f'Motor turns callback, current_y: {self.current_y}')
            self.condition.notify_all()

    def imu_callback(self, msg):
        with self.lock:
            self.imu_angle = msg.data
            self.get_logger().info(f'IMU callback, imu_angle: {self.imu_angle}')
            self.condition.notify_all()

    def automation_callback(self, msg):
        with self.lock:
            self.automation_active = msg.data
            self.get_logger().info(f'Automation callback, automation_active: {self.automation_active}')
            self.condition.notify_all()
        if not self.automation_active:
            self.publish_velocity(0.0, 0.0)
        else:
            self.get_logger().info('Automation activated')

    def control_loop(self):
        while rclpy.ok():
            with self.lock:
                while not self.automation_active:
                    self.condition.wait()
                self.get_logger().info('Automation loop started')
                while self.automation_active and not self.apriltag_left and not self.apriltag_right:
                    self.get_logger().info('Start loop')
                    self.get_logger().info('Drive down')
                    self.drive_y(self.down_speed, 0.7)
                    self.get_logger().info('Drive up')
                    self.drive_y(self.up_speed, 0)
                    self.get_logger().info('Turn')
                    self.turn_to_angle(self.turn_speed, 0.5)
                    self.get_logger().info('Drive cross')
                    self.drive_time(self.cross_speed, 3, 0.5)
                    self.get_logger().info('Turn')
                    self.turn_to_angle(self.turn_speed, 0)
                    self.get_logger().info('Loop end')
                self.get_logger().info('Loop out')
                self.get_logger().info('Drive down last time')
                self.drive_y(self.down_speed, 0.7)
                self.get_logger().info('Drive up last time')
                self.drive_y(self.up_speed, 0)
                self.get_logger().info('Turn to the other side')
                self.turn_to_angle(self.turn_speed, -0.5)
                self.get_logger().info('Drive home')
                self.drive_time(self.cross_speed, 20, -0.5)
                self.get_logger().info('Turn to homing position')
                self.turn_to_angle(self.turn_speed, 0)

    def drive_straight(self, velocity, imu_goal):
        with self.lock:
            if not self.automation_active:
                return
            error = imu_goal - self.imu_angle
            twist = Twist()
            twist.linear.x = velocity
            twist.angular.z = self.turn_speed if error > 0 else -self.turn_speed
            self.cmd_vel_publisher.publish(twist)

    def turn_to_angle(self, velocity, imu_goal):
        while True:
            with self.lock:
                if not self.automation_active:
                    return
                error = imu_goal - self.imu_angle
                if abs(error) <= 0.05:
                    break
                twist = Twist()
                twist.angular.z = velocity if error > 0 else -velocity
                self.cmd_vel_publisher.publish(twist)
            self.condition.wait(timeout=0.04)  # Add a timeout for the condition wait to periodically check the error
        self.publish_velocity(0.0, 0.0)  # Stop turning when angle is reached

    def drive_y(self, velocity, y_goal):
        while True:
            with self.lock:
                if abs(self.current_y - y_goal) <= 0.05 or not self.automation_active:
                    break
                self.get_logger().info(f'Driving to y_goal: {y_goal}, current_y: {self.current_y}')
                self.drive_straight(velocity, 0.0)
            self.condition.wait(timeout=0.04)  # Add a timeout for the condition wait to periodically check the y position
        self.publish_velocity(0.0, 0.0)  # Stop when the goal is reached

    def drive_time(self, velocity, time, goal_angle):
        start_time = self.get_clock().now()
        while True:
            with self.lock:
                if (self.get_clock().now() - start_time).seconds >= time or not self.automation_active or self.apriltag_left or self.apriltag_right:
                    break
                self.drive_straight(velocity, goal_angle)
            self.condition.wait(timeout=0.04)  # Add a timeout for the condition wait to periodically check the time
        self.publish_velocity(0.0, 0.0)  # Stop when the goal is reached or time is over

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
