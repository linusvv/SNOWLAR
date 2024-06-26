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

        
        # Publishers for each winch motor
        self.pub_right = self.create_publisher(Float32, "/olive/servo/wr/goal/velocity", QoSProfile(depth=10))
        self.pub_left = self.create_publisher(Float32, "/olive/servo/wl/goal/velocity", QoSProfile(depth=10))
        
        # Subscription to winch and base_to_winch
        self.sub_winch = self.create_subscription(Twist, "/winch", self.callback_winch, QoSProfile(depth=10)) 
        self.sub_base_to_winch = self.create_subscription(Twist, "/base_to_winch", self.callback_base_to_winch, QoSProfile(depth = 10))
        ##self.sub_lef_vel = self.create_subscription(Twist, "winch", self.callback_cmd_vel, QoSProfile(depth=10))


        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25.0
        
        self.velocity_right = 0.0
        self.velocity_left = 0.0

        self.target_velocity_right = 0.0
        self.target_velocity_left = 0.0

        self.max_velocity = 3.0
        
        self.alpha = 0.9  # Low-pass filter constant (0 < alpha <= 1)
        
        self.thread_main.start()

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:
            # Apply low-pass filter to smooth velocity changes
            self.velocity_right = self.low_pass_filter(self.velocity_right, self.target_velocity_right)
            self.velocity_left = self.low_pass_filter(self.velocity_left, self.target_velocity_left)
            
            # Publish the velocities for each winch motor
            self.publish_velocity(self.pub_left, -1*self.velocity_right)        #somehow, the order was inverted, maybe not?
            self.publish_velocity(self.pub_right, -1*self.velocity_left)
            
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

    def callback_winch(self, msg):
        vel_Left = msg.linear.x    # manual mode left/right
        vel_Right = msg.linear.y   # manual mode up/down

        self.target_velocity_right = vel_Right 
        self.target_velocity_left = vel_Left 



    def callback_base_to_winch(self, msg):

        self.chainLeft = msg.linear.x       #Velocity left chain update
        self.chainRight = msg.linear.y      #Velocity right chain update
    
        

    def __del__(self):
        self.thread_exited = True
        if self.thread_main.is_alive():
            self.thread_main.join()

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode("config/path", "main_node")
    print("whinch control active")
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
        main()
