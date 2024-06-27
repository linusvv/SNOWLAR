import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import threading
import time
import math

imu_data_lock = threading.Lock()
imu_data = 0.0

class MotorCalibrationNode(Node):
    def __init__(self):
        super().__init__('motor_calibration_node')
        self.pub_calib = self.create_publisher(Float32, "/olive/servo/calib/goal/position", QoSProfile(depth=10))
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(SetBool, 'calibrate_motor', self.calibrate_motor_callback)
        self.subscription_imu_data = self.create_subscription(
            Float32,
            '/imu_data',
            self.imu_data_callback,
            10
        )

        self.imu_reset_client = self.create_client(Trigger, 'reset_imu_data')
        self.imu_lock_client = self.create_client(Trigger, 'lock_imu_data')
        self.imu_unlock_client = self.create_client(Trigger, 'unlock_imu_data')

    def calibrate_motor_callback(self, request, response):
        if request.data:
            self.get_logger().info('Calibration started...')

            # Unlock IMU data
            self.reset_imu_data()
            time.sleep(1)
            # Rotate to -10 degrees
            self.publish_motor_position(-0.6 * math.pi)
            time.sleep(2)  # Adjust sleep time as necessary
            
            # Rotate to 190 degrees
            self.publish_motor_position(0.6 * math.pi)
            time.sleep(2)  # Adjust sleep time as necessary
            
            # Rotate back to zero position
            self.publish_motor_position(-0.5 * math.pi)
            time.sleep(2)  # Adjust sleep time as necessary

            self.get_logger().info('IMU Calibration completed.')

            # Calibrate wheels
            # self.calibrate_wheels()

            # Lock IMU data
            time.sleep(1)
            self.lock_imu_data()

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

    def reset_imu_data(self):
        while not self.imu_reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IMU reset service...')
        
        request = Trigger.Request()
        future = self.imu_reset_client.call_async(request)
        future.add_done_callback(self.reset_imu_data_response)

    def reset_imu_data_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('IMU data reset successfully.')
            else:
                self.get_logger().warn('Failed to reset IMU data.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def lock_imu_data(self):
        while not self.imu_lock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IMU lock service...')
        
        request = Trigger.Request()
        future = self.imu_lock_client.call_async(request)
        future.add_done_callback(self.lock_imu_data_response)

    def lock_imu_data_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('IMU data locked successfully.')
            else:
                self.get_logger().warn('Failed to lock IMU data.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def unlock_imu_data(self):
        while not self.imu_unlock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IMU unlock service...')
        
        request = Trigger.Request()
        future = self.imu_unlock_client.call_async(request)
        future.add_done_callback(self.unlock_imu_data_response)

    def unlock_imu_data_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('IMU data unlocked successfully.')
            else:
                self.get_logger().warn('Failed to unlock IMU data.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def calibrate_wheels(self):
        self.get_logger().info('Starting wheel calibration...')

        rate = self.create_rate(50)  # Set rate to 50 Hz

        # Rotate left until IMU data rises
        twist_msg = Twist()
        twist_msg.linear.y = 0.3  # Adjust speed as necessary

        with imu_data_lock:
            previous_imu_data = imu_data
        
        while True:
            with imu_data_lock:
                current_imu_data = imu_data

            if current_imu_data > previous_imu_data:
                break

            previous_imu_data = current_imu_data
            self.publisher_cmd_vel.publish(twist_msg)
            rate.sleep()
        
        # Rotate right until IMU data reaches minimum
        twist_msg.linear.y = -0.3  # Adjust speed as necessary

        min_imu_data = current_imu_data

        while True:
            self.publisher_cmd_vel.publish(twist_msg)
            rate.sleep()
            
            with imu_data_lock:
                current_imu_data = imu_data

            if current_imu_data < min_imu_data:
                min_imu_data = current_imu_data
            elif current_imu_data > min_imu_data:
                break
        
        # Stop rover
        twist_msg.linear.y = 0.0
        for _ in range(10):  # Publish stop command for a short duration to ensure rover stops
            self.publisher_cmd_vel.publish(twist_msg)
            rate.sleep()

        self.get_logger().info('Wheel calibration completed.')

def main(args=None):
    rclpy.init(args=args)
    node = MotorCalibrationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
