import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Int32, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AprilTagSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_subscriber')
        
        # Create QoS profile with RELIABILITY policy set to BEST_EFFORT
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/olive/camera/camera/tags',
            self.listener_callback,
            qos_profile)
        
        self.id_publisher = self.create_publisher(Int32, '/apriltag_id', 10)
        self.stop_publisher = self.create_publisher(Bool, '/camera_stop', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        detected = bool(msg.detections)
        stop_msg = Bool()
        stop_msg.data = detected
        self.stop_publisher.publish(stop_msg)
        
        if detected:
            first_tag_id = msg.detections[0].id
            self.get_logger().info(f'First AprilTag ID: {first_tag_id}')
            id_msg = Int32()
            id_msg.data = first_tag_id
            self.id_publisher.publish(id_msg)

def main(args=None):
    rclpy.init(args=args)
    apriltag_subscriber = AprilTagSubscriber()
    rclpy.spin(apriltag_subscriber)
    apriltag_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
