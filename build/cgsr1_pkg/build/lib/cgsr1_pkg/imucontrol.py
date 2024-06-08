import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf2_ros
import math

class IMUCalibrationNode(Node):
    def __init__(self):
        super().__init__('imu_calibration_node')
        self.max_yaw = None
        self.min_yaw = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_topic',  # Replace with your IMU topic
            self.imu_callback,
            10
        )

        self.house_angle_pub = self.create_publisher(
            Float64,
            '/house_angle',
            10
        )

    def imu_callback(self, msg):
        try:
            # Get transform from base_link to imu_link
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'imu_link', rclpy.time.Time())
            
            # Extract yaw from quaternion
            orientation = transform.transform.rotation
            roll, pitch, yaw = self.quaternion_to_euler(
                orientation.x, orientation.y, orientation.z, orientation.w)

            # Update max and min yaw during calibration phase
            if self.max_yaw is None or yaw > self.max_yaw:
                self.max_yaw = yaw
            if self.min_yaw is None or yaw < self.min_yaw:
                self.min_yaw = yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn("Failed to lookup transform: %s" % str(e))

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def calculate_house_angle(self):
        if self.max_yaw is None or self.min_yaw is None:
            return None

        # Calculate angle of the house with respect to the ridge
        house_angle = (self.max_yaw + self.min_yaw) / 2.0

        return house_angle

    def publish_house_angle(self):
        house_angle = self.calculate_house_angle()
        if house_angle is not None:
            msg = Float64()
            msg.data = house_angle
            self.house_angle_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_calibration_node = IMUCalibrationNode()

    # Run calibration phase for some time
    rclpy.spin_once(imu_calibration_node)
    imu_calibration_node.publish_house_angle()

    imu_calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
