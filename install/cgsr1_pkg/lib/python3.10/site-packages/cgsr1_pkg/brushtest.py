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
        
        # Publisher for the brush motor PWM
        self.pub_brush = self.create_publisher(Float32, "/olive/servo/brush/pwm", QoSProfile(depth=10))
        
        # Subscription to cmd_vel
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.callback_cmd_vel, QoSProfile(depth=10))

        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 40
        
        self.duty_cycle_brush = 0.0
        self.target_velocity_brush = 0.0

        self.max_velocity = -20.0
        
        self.thread_main.start()

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:
            # Publish the duty cycle for the brush motor PWM
            self.publish_duty_cycle(self.pub_brush, self.duty_cycle_brush)
            
            time.sleep(1 / self.rate_control_hz)

    def publish_duty_cycle(self, publisher, duty_cycle):
        msg = Float32()
        # Convert velocity to duty cycle (assuming linear mapping)
        if abs(duty_cycle) < 0.1:
            msg.data = 0.0
        else:
            msg.data = duty_cycle
        publisher.publish(msg)

    def callback_cmd_vel(self, msg):
        # Map linear velocity to duty cycle
        if abs(msg.linear.x) > 0.1:
            self.duty_cycle_brush = self.max_velocity
        elif msg.linear.y > 0.1:
            self.duty_cycle_brush = self.max_velocity
        else:
            self.duty_cycle_brush = 0.0

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
