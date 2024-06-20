import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

# Define the diameter of the winches as a global variable (in meters)
WINCH_DIAMETER = 0.047  # Example diameter in meters, change as necessary
# Calculate the circumference of the winches
DRUM_CIRCUMFERENCE = math.pi * WINCH_DIAMETER

class JointStateMotorTurnCounter(Node):
    def __init__(self):
        super().__init__('joint_state_motor_turn_counter')

        self.motor1_subscription = self.create_subscription(
            JointState,
            '/olive/servo/wl/joint/state',
            self.motor1_callback,
            10)
        
        self.motor2_subscription = self.create_subscription(
            JointState,
            '/olive/servo/wr/joint/state',
            self.motor2_callback,
            10)

        self.zero_subscription = self.create_subscription(
            Bool,
            '/set_winch_zero',
            self.set_zero_callback,
            10)
        
        self.publisher = self.create_publisher(Twist, 'motor_turns', 10)
        
        # Initialize variables to keep track of turns and distance
        self.last_position = [0.0, 0.0]
        self.turns = [0.0, 0.0]
        self.first_message = [True, True]
        self.distance_pulled = [0.0, 0.0]  # Distance pulled for each winch

    def motor1_callback(self, msg):
        if len(msg.position) > 0:
            self.update_turns(0, msg.position[0])

    def motor2_callback(self, msg):
        if len(msg.position) > 0:
            self.update_turns(1, msg.position[0])

    def update_turns(self, motor_index, current_position):
        if self.first_message[motor_index]:
            self.last_position[motor_index] = current_position
            self.first_message[motor_index] = False
            return
        
        diff = current_position - self.last_position[motor_index]
        if diff > math.pi:  # If the difference is more than pi, it means it crossed 0
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        self.turns[motor_index] += diff / (2 * math.pi)
        
        self.last_position[motor_index] = current_position

        # Calculate distance pulled based on turns
        self.distance_pulled = (self.turns[0]* WINCH_DIAMETER * math.pi - self.turns[1]* WINCH_DIAMETER * math.pi)/2 

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = self.turns[0] 
        twist.linear.y = self.turns[1]
        twist.linear.z = self.distance_pulled  # Distance pulled by motor 1
        self.publisher.publish(twist)
        
        # Log the number of turns and distance pulled
        self.get_logger().info(f'Motor 1 turns: {self.turns[0]}')
        self.get_logger().info(f'Motor 2 turns: {self.turns[1]}')

    def set_zero_callback(self, msg):
        if msg.data:
            self.turns = [0.0, 0.0]
            self.distance_pulled = [0.0, 0.0]
            self.get_logger().info('Turns and distance pulled have been reset to zero.')

def main(args=None):
    rclpy.init(args=args)
    motor_turn_counter = JointStateMotorTurnCounter()
    print("JointState motor turn counter active")
    rclpy.spin(motor_turn_counter)
    motor_turn_counter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
