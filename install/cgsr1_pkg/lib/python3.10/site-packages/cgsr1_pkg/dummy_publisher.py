import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import random

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        
        # Publisher for the GUI dimensions (width, height)
        self.publisher_dimensions = self.create_publisher(Int32MultiArray, '/gui_dimensions', 10)
        
        # Publisher for the GUI position
        self.publisher_position = self.create_publisher(Float32MultiArray, '/gui_position', 10)
        
        # Publish dimensions once
        self.publish_dimensions()

        # Publish position periodically
        self.timer = self.create_timer(1.0, self.publish_position)  # Publish every second

    def publish_dimensions(self):
        msg = Int32MultiArray()
        msg.data = [1000, 10]  # Example values for width and height
        self.publisher_dimensions.publish(msg)
        self.get_logger().info(f'Published dimensions: width=300, height=150')

    def publish_position(self):
        msg = Float32MultiArray()
        dot_x = random.uniform(0, 300 - 20)
        dot_y = random.uniform(0, 150 - 20)
        msg.data = [dot_x, dot_y]
        self.publisher_position.publish(msg)
        self.get_logger().info(f'Published position: x={dot_x}, y={dot_y}')

def main(args=None):
    rclpy.init(args=args)
    dummy_publisher = DummyPublisher()
    rclpy.spin(dummy_publisher)
    dummy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
