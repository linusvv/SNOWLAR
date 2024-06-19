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
        
        # Publisher for the brush motor
        self.pub_brush = self.create_publisher(Float32, "/olive/servo/brush/goal/velocity", QoSProfile(depth=10))
        
        # Subscription to cmd_vel
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.callback_cmd_vel, QoSProfile(depth=10))

        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 40
        
        self.velocity_brush = 0.0
        self.target_velocity_brush = 0.0

        self.max_velocity = -100.0
        
        self.alpha = 0.1  # Low-pass filter constant (0 < alpha <= 1)
        
        self.thread_main.start()

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:
            # Apply low-pass filter to smooth velocity changes
            self.velocity_brush = self.low_pass_filter(self.velocity_brush, self.target_velocity_brush)
            
            # Publish the velocity for the brush motor
            self.publish_velocity(self.pub_brush, self.velocity_brush)
            
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
        if msg.linear.x > 0.1: # Linear velocity in x-direction
            self.target_velocity_brush = self.max_velocity
        elif abs(msg.linear.y) > 0.1:
            self.target_velocity_brush = self.max_velocity

    def __del__(self):
        self.thread_exited = True
        if self.thread_main.is_alive():
            self.thread_main.join()

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode("config/path", "main_node")
    print("brush motor control active")
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
