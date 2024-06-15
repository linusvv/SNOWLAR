# motor_calibration_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool
from std_msgs.msg import Float32
import threading
import time

imu_data_lock = threading.Lock()
imu_data = 0.0

class MotorCalibrationNode(Node):
    def __init__(self):
        super().__init__('motor_calibration_node')
        self.pub_calib = self.create_publisher(Float32, "/olive/servo/calib/goal/position", QoSProfile(depth=10))
        self.srv = self.create_service(SetBool, 'calibrate_motor', self.calibrate_motor_callback)
        self.subscription_imu_data = self.create_subscription(
            Float32,
            '/imu_data',
            self.imu_data_callback,
            10
        )

    def calibrate_motor_callback(self, request, response):
        if request.data:
            self.get_logger().info('Calibration started...')
            
            # Rotate to -10 degrees
            self.publish_motor_position(-10.0)
            time.sleep(1)  # Adjust sleep time as necessary
            
            # Rotate to 190 degrees
            self.publish_motor_position(190.0)
            time.sleep(1)  # Adjust sleep time as necessary
            
            # Rotate back slowly until IMU data is less than 0.001
            self.rotate_back_slowly()

            self.get_logger().info('Calibration completed.')
            response.success = True
            response.message = 'Calibration successful'
        else:
            self.get_logger().info('Calibration not started. Received false request.')
            response.success = False
            response.message = 'Calibration not started'
        return response

    def imu_data_callback(self, msg):
        global imu_data
        with imu_data_lock:
            imu_data = msg.data

    def publish_motor_position(self, position):
        msg = Float32()
        msg.data = float(position)
        self.pub_calib.publish(msg)
        self.get_logger().info(f'Motor position set to: {position}')

    def rotate_back_slowly(self):
        global imu_data
        position = 190.0
        step = -0.1  # Small step for slow rotation

        while position > -10:
            with imu_data_lock:
                if imu_data < 0.001:
                    self.get_logger().info('IMU data is less than 0.001. Stopping...')
                    break
            
            position += step
            self.publish_motor_position(position)
            time.sleep(0.1)  # Adjust sleep time for slower or faster rotation

def main(args=None):
    rclpy.init(args=args)
    node = MotorCalibrationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
