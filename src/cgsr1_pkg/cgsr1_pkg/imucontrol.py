import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

class IMUDirectionNode(Node):

    def __init__(self):
        super().__init__('imu_direction_node')
        
        self.subscription = self.create_subscription(
            Imu,
            '/olive/imu/imu/filtered_ahrs',
            self.imu_callback,
            10)
        
        self.publisher_ = self.create_publisher(Float64, 'imu/driving_direction', 10)
        self.max_pitch = None  # Initialize the max_pitch as None
        self.min_pitch = None  # Initialize the min_pitch as None
        
        self.get_logger().info('IMU Direction Node has been started.')

    def imu_callback(self, msg):
        # Extract the orientation quaternion from the IMU message
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation)

        # Update the maximum and minimum pitch observed
        if self.max_pitch is None or pitch > self.max_pitch:
            self.max_pitch = pitch
        if self.min_pitch is None or pitch < self.min_pitch:
            self.min_pitch = pitch

        # Normalize pitch based on the current maximum and minimum pitch
        if self.max_pitch != self.min_pitch:  # Avoid division by zero
            normalized_pitch = (pitch - self.min_pitch) / (self.max_pitch - self.min_pitch)
            # Scale normalized pitch to the range [-1, 1] and add sign
            signed_normalized_pitch = 2 * (normalized_pitch - 0.5)
        else:
            signed_normalized_pitch = 0.0

        self.get_logger().info(f'Signed Normalized Pitch: {signed_normalized_pitch}, Yaw: {yaw}')
        
        # Publish the signed normalized pitch
        driving_direction = Float64()
        driving_direction.data = signed_normalized_pitch
        self.publisher_.publish(driving_direction)

    def quaternion_to_euler(self, orientation):
        # Convert quaternion to Euler angles
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = IMUDirectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
