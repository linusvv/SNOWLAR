import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import threading
import time
import math

class MainNode(Node):
    def __init__(self, config_path, node_name):
        super().__init__(node_name)
        
        # Publishers for each wheel


        
        self.pub_cmd_vel_automated = self.create_publisher(Twist, '/cmd_vel_automated', QoSProfile(depth=10))

        # Subscription for IMU data
        self.subscription_imu_data = self.create_subscription(
            Float32,
            '/imu_data',
            self.imu_data_callback,
            QoSProfile(depth=10)
        )

        # Subscription to cmd_vel
        self.sub_cmd_vel = self.create_subscription(Twist, "/cmd_vel_straight", self.callback_cmd_vel_straight, QoSProfile(depth=10))  # Linear x is velocity, linear y is goal angle

        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25
        
        


        self.alpha = 0.1  # Low-pass filter constant (0 < alpha <= 1)

        self.current_imu_angle = 0.0
        self.target_imu_angle = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.tolerance = 0.05
        self.max_velocity = 3.0


        self.thread_main.start()

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:
            # Calculate steering adjustment based on IMU data
            self.velocity_y = self.calculate_steering_adjustment()

            
            self.publish_cmd_vel_automated(self.velocity_x, self.velocity_y)

            
            time.sleep(1 / self.rate_control_hz)

    def low_pass_filter(self, current_velocity, target_velocity):
        return current_velocity + self.alpha * (target_velocity - current_velocity)

    def publish_velocity(self, publisher, velocity):
        msg = Float32()
        if abs(velocity) < 0.1:
            msg.data = 0.0
        else:
            msg.data = velocity
        publisher.publish(msg)

    def imu_data_callback(self, msg):
        self.current_imu_angle = msg.data

    def calculate_steering_adjustment(self):
        angle_error = self.target_imu_angle - self.current_imu_angle
        k_p = 10  # Proportional gain for steering adjustment
        if abs(angle_error) > self.tolerance and abs(angle_error) < 2-self.tolerance:
            steering_adjustment = k_p * angle_error
        else: steering_adjustment = 0.0
        return steering_adjustment

    def callback_cmd_vel_straight(self, msg):
        vx = msg.linear.x  # Linear velocity in x-direction
        self.target_imu_angle = msg.linear.y  # Goal angle from IMU
        
        vx = vx * self.max_velocity
        self.velocity_x = vx

    def publish_cmd_vel_automated(self, vx,vy):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy

        self.pub_cmd_vel_automated.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode("config/path", "main_node")
    print("base control active")
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
