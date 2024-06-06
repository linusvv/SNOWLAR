import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import threading
import time

class MainNode(Node):
    def __init__(self, config_path, node_name):
        super().__init__(node_name)
        
        # Publishers for each winch motor
        self.pub_right = self.create_publisher(Float32, "/olive/servo/wr/goal/velocity", QoSProfile(depth=10))
        self.pub_left = self.create_publisher(Float32, "/olive/servo/wl/goal/velocity", QoSProfile(depth=10))
        
        # Subscription to cmd_vel
        self.sub_cmd_vel = self.create_subscription(Twist, "winch", self.callback_cmd_vel, QoSProfile(depth=10))

        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25
        
        self.velocity_right = 0.0
        self.velocity_left = 0.0

        self.target_velocity_right = 0.0
        self.target_velocity_left = 0.0

        self.max_velocity = 4
        
        self.alpha = 0.1  # Low-pass filter constant (0 < alpha <= 1)
        
        self.thread_main.start()

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:
            # Apply low-pass filter to smooth velocity changes
            self.velocity_right = self.low_pass_filter(self.velocity_right, self.target_velocity_right)
            self.velocity_left = self.low_pass_filter(self.velocity_left, self.target_velocity_left)
            
            # Publish the velocities for each winch motor
            self.publish_velocity(self.pub_right, self.velocity_right)
            self.publish_velocity(self.pub_left, -self.velocity_left)
            
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

    def callback_cmd_vel(self, msg):
        vx = msg.linear.x  # Linear velocity for the first winch motor
        vy = msg.linear.y  # Linear velocity for the second winch motor

        # Scale the velocities according to max_velocity
        vx = vx * self.max_velocity
        vy = vy * self.max_velocity

        # Debug prints to check the scaled velocities
        print(f'Received linear velocities - x (winch 1): {msg.linear.x}, y (winch 2): {msg.linear.y}')
        print(f'Scaled velocities - vx: {vx}, vy: {vy}')
        
        # Set the target velocities for the winch motors directly
        self.target_velocity_left = vx  # First winch motor
        self.target_velocity_right = vy  # Second winch motor

        # Debug prints to confirm target velocities are set correctly
        print(f'Target velocities - Left (winch 1): {self.target_velocity_left}, Right (winch 2): {self.target_velocity_right}')

    def __del__(self):
        self.thread_exited = True
        if self.thread_main.is_alive():
            self.thread_main.join()

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode("config/path", "main_node")
    print("Bots_Bento_Base_Control_v1.0")
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
