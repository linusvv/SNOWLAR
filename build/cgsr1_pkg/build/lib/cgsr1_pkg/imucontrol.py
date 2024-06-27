import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import math

class IMUDirectionNode(Node):

    def __init__(self):
        super().__init__('imu_direction_node')
        
        self.subscription = self.create_subscription(
            Imu,
            '/olive/imu/imu/filtered_ahrs',
            self.imu_callback,
            10)
        
        self.publisher_ = self.create_publisher(Float32, 'imu_data', 10)
        self.max_pitch = None  # Initialize the max_pitch as None
        self.min_pitch = None  # Initialize the min_pitch as None

        self.locked = False
        
        self.reset_service = self.create_service(Trigger, 'reset_imu_data', self.reset_imu_data_callback)
        self.lock_service = self.create_service(Trigger, 'lock_imu_data', self.lock_imu_data_callback)
        self.unlock_service = self.create_service(Trigger, 'unlock_imu_data', self.unlock_imu_data_callback)

        self.get_logger().info('IMU Direction Node has been started.')

    def imu_callback(self, msg):

        
        # Extract the orientation quaternion from the IMU message
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation)

        # Update the maximum and minimum pitch observed
        if not self.locked:
            if self.max_pitch is None or pitch > self.max_pitch:
                self.max_pitch = pitch
            if self.min_pitch is None or pitch < self.min_pitch:
                self.min_pitch = pitch

        # Normalize pitch based on the current maximum and minimum pitch
        if self.max_pitch != self.min_pitch:  # Avoid division by zero
            normalized_pitch = (pitch - self.min_pitch) / (self.max_pitch - self.min_pitch)
        else:
            normalized_pitch = 0.0

        # Determine the sign based on the roll angle
        pitch_sign = math.copysign(1, roll)

        # Apply the sign to the normalized pitch
        signed_normalized_pitch = normalized_pitch * pitch_sign

        self.get_logger().info(f'Signed Normalized Pitch: {signed_normalized_pitch}, Roll: {roll}, Yaw: {yaw}')
        
        # Publish the signed normalized pitch
        driving_direction = Float32()
        driving_direction.data = signed_normalized_pitch
        self.publisher_.publish(driving_direction)

    def reset_imu_data_callback(self, request, response):
        self.max_pitch = None  # Reset the max_pitch
        self.min_pitch = None  # Reset the min_pitch
        response.success = True
        self.locked = False
        response.message = "IMU data has been reset."
        self.get_logger().info(response.message)
        return response

    def lock_imu_data_callback(self, request, response):
        self.locked = True
        response.success = True
        response.message = "IMU data has been locked."
        self.get_logger().info(response.message)
        return response

    def unlock_imu_data_callback(self, request, response):
        self.locked = False
        response.success = True
        response.message = "IMU data has been unlocked."
        self.get_logger().info(response.message)
        return response

    def quaternion_to_euler(self, q):
        # Convert quaternion to Euler angles
        x, y, z, w = q.x, q.y, q.z, q.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    node = IMUDirectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
