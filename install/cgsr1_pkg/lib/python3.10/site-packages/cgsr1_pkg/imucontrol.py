import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import sys  # Add this import statement

class IMUDirectionNode(Node):

    def __init__(self, roof_angle):
        super().__init__('imu_direction_node')
        
        self.subscription = self.create_subscription(
            Imu,
            '/olive/imu/imu/filtered_ahrs',
            self.imu_callback,
            10)
        
        self.publisher_ = self.create_publisher(Float64, 'imu/driving_direction', 10)
        self.max_pitch = -roof_angle  # Maximum pitch is negative of the roof angle
        
        self.get_logger().info('IMU Direction Node has been started.')

    def imu_callback(self, msg):
        # Extract the orientation quaternion from the IMU message
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation)

        # Correct pitch based on the maximum pitch
        corrected_pitch = pitch - self.max_pitch

        self.get_logger().info(f'Corrected Pitch: {corrected_pitch}, Yaw: {yaw}')
        
        # Publish the driving direction (yaw)
        driving_direction = Float64()
        driving_direction.data = yaw
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
    
    # Provide the roof angle as a command line argument
    if len(sys.argv) != 2:
        print("Usage: ros2 run imu_direction imu_direction_node <roof_angle>")
        return

    roof_angle = float(sys.argv[1])

    node = IMUDirectionNode(roof_angle)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
