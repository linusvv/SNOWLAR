import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Vector3  # Using Vector3 for map size

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        
        # Publisher for map size
        self.map_size_pub = self.create_publisher(Vector3, 'map_topic', 10)
        
        # Publisher for robot position
        self.position_pub = self.create_publisher(Pose, 'robot_position_topic', 10)
        
        # Timer to publish at regular intervals
        self.timer = self.create_timer(1.0, self.publish_messages)  # 1 second interval

    def publish_messages(self):
        # Dummy map size data
        map_size = Vector3()
        map_size.x = 600.0  # width
        map_size.y = 600.0  # height

        # Dummy robot position data
        robot_position = Pose()
        robot_position.position.x = 300.0
        robot_position.position.y = 300.0

        # Publishing the messages
        self.map_size_pub.publish(map_size)
        self.position_pub.publish(robot_position)

        self.get_logger().info('Publishing: Map size (width, height): ({}, {}), Robot position (x, y): ({}, {})'.format(
            map_size.x, map_size.y, robot_position.position.x, robot_position.position.y))

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
