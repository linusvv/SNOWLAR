import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class MotorTurnCounter(Node):
    def __init__(self):
        super().__init__('motor_turn_counter')

        self.motor1_subscription = self.create_subscription(
            Float64,
            'motor1_position',
            self.motor1_callback,
            10)
        
        self.motor2_subscription = self.create_subscription(
            Float64,
            'motor2_position',
            self.motor2_callback,
            10)
        
        self.publisher = self.create_publisher(Twist, 'motor_turns', 10)
        
        # Initialize variables to keep track of turns
        self.last_position = [0.0, 0.0]
        self.turns = [0.0, 0.0]
        self.first_message = [True, True]

    def motor1_callback(self, msg):
        self.update_turns(0, msg.data)

    def motor2_callback(self, msg):
        self.update_turns(1, msg.data)

    def update_turns(self, motor_index, current_position):
        if self.first_message[motor_index]:
            self.last_position[motor_index] = current_position
            self.first_message[motor_index] = False
            return
        
        diff = current_position - self.last_position[motor_index]
        if diff > 3.14159:  # If the difference is more than pi, it means it crossed 0
            diff -= 2 * 3.14159
        elif diff < -3.14159:
            diff += 2 * 3.14159
        self.turns[motor_index] += diff / (2 * 3.14159)
        
        self.last_position[motor_index] = current_position

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = self.turns[0]
        twist.linear.y = self.turns[1]
        self.publisher.publish(twist)
        self.get_logger().info(f'Motor 1 turns: {self.turns[0]}, Motor 2 turns: {self.turns[1]}')

def main(args=None):
    rclpy.init(args=args)
    motor_turn_counter = MotorTurnCounter()
    rclpy.spin(motor_turn_counter)
    motor_turn_counter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
