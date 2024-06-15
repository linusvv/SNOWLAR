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
        self.pub_front_right = self.create_publisher(Float32, "/olive/servo/mfr/goal/velocity", QoSProfile(depth=10))
        self.pub_front_left = self.create_publisher(Float32, "/olive/servo/mfl/goal/velocity", QoSProfile(depth=10))
        self.pub_rear_right = self.create_publisher(Float32, "/olive/servo/mbr/goal/velocity", QoSProfile(depth=10))
        self.pub_rear_left = self.create_publisher(Float32, "/olive/servo/mbl/goal/velocity", QoSProfile(depth=10))

        self.pub_mimu = self.create_publisher(Float32, "/olive/servo/mimu/goal/velocity", QoSProfile(depth=10))


        self.base_to_winch = self.create_publisher(Twist, "/base_to_winch", QoSProfile(depth=10))
        
        # Subscription to cmd_vel
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.callback_cmd_vel, QoSProfile(depth=10))

        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 40
        
        self.velocity_front_right = 0.0
        self.velocity_front_left = 0.0
        self.velocity_rear_right = 0.0
        self.velocity_rear_left = 0.0

        self.target_velocity_front_right = 0.0
        self.target_velocity_front_left = 0.0
        self.target_velocity_rear_right = 0.0
        self.target_velocity_rear_left = 0.0

        self.max_velocity = 3
        
        self.alpha = 0.1  # Low-pass filter constant (0 < alpha <= 1)
        
        self.thread_main.start()

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:
            # Apply low-pass filter to smooth velocity changes
            self.velocity_front_right = self.low_pass_filter(self.velocity_front_right, self.target_velocity_front_right)
            self.velocity_front_left = self.low_pass_filter(self.velocity_front_left, self.target_velocity_front_left)
            self.velocity_rear_right = self.low_pass_filter(self.velocity_rear_right, self.target_velocity_rear_right)
            self.velocity_rear_left = self.low_pass_filter(self.velocity_rear_left, self.target_velocity_rear_left)
            
            # Publish the velocities for each wheel
            self.publish_velocity(self.pub_front_right, self.velocity_front_right)
            self.publish_velocity(self.pub_front_left, self.velocity_front_left)
            self.publish_velocity(self.pub_rear_right, self.velocity_rear_right)
            self.publish_velocity(self.pub_rear_left, self.velocity_rear_left)
            self.publish_velocity(self.pub_mimu, self.velocity_mimu)

            self.publish_base_to_winch(self.base_to_winch, self.velocity_front_left, self.velocity_front_right)
            
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

    def publish_base_to_winch(self, publisher, dataX, dataY):
        msg = Twist()
        msg.linear.x = dataX
        msg.linear.y = dataY

        publisher.publish(msg)   

    def callback_cmd_vel(self, msg):
        vx = msg.linear.x  # Linear velocity in x-direction
        vy = msg.linear.y  # Linear velocity in y-direction
        
        vx = vx * self.max_velocity
        vy = vy * self.max_velocity

        print(f'This is the x velocity: {vx}')

        self.target_velocity_front_left = vx - vy
        self.target_velocity_front_right = -1*(vx+vy)
        self.target_velocity_rear_left = self.target_velocity_front_left
        self.target_velocity_rear_right = self.target_velocity_front_right

        self.velocity_mimu = msg.angular.x

    def __del__(self):
        self.thread_exited = True
        if self.thread_main.is_alive():
            self.thread_main.join()

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode("config/path", "main_node")
    print("base control active")
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
