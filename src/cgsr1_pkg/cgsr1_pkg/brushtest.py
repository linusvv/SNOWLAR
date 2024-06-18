import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.publisher_ = self.create_publisher(String, 'motor_status', 10)
        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.motor_running = False
        self.get_logger().info('Motor Controller has been started')

    def command_callback(self, msg):
        command = msg.data.lower()
        if command == 'start':
            self.start_motor()
        elif command == 'stop':
            self.stop_motor()

    def start_motor(self):
        if not self.motor_running:
            self.motor_running = True
            self.publish_motor_status('Motor started')
            self.get_logger().info('Motor started')

    def stop_motor(self):
        if self.motor_running:
            self.motor_running = False
            self.publish_motor_status('Motor stopped')
            self.get_logger().info('Motor stopped')

    def publish_motor_status(self, status):
        msg = String()
        msg.data = status
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
