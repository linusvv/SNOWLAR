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

        self.angle = 0.0
        self.chainRight = 0.0
        self.chainLeft = 0.0
        self.translation_Factor = 98/47 #translation factor between big chain wheels and winch wheels
        
        # Publishers for each winch motor
        self.pub_right = self.create_publisher(Float32, "/olive/servo/wr/goal/velocity", QoSProfile(depth=10))
        self.pub_left = self.create_publisher(Float32, "/olive/servo/wl/goal/velocity", QoSProfile(depth=10))
        
        # Subscription to cmd_vel
        self.sub_winch = self.create_subscription(Twist, "/winch", self.callback_winch, QoSProfile(depth=10))
        self.sub_base_to_winch = self.create_subsciption(Twist, "/base_to_winch", self.callback_base_to_winch, QoSProfile(depth = 10))
        ##self.sub_lef_vel = self.create_subscription(Twist, "winch", self.callback_cmd_vel, QoSProfile(depth=10))


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
        vel_Left = msg.angular.x    # manual mode left
        vel_Right = msg.angular.y   # manual mode right
        tempAngle = (msg.angular.z + 1) * math.pi # Angle



        if vel_Left == 0 and vel_Right == 0:
            self.velocity_left = self.translation_Factor(math.cos(self.angle)* self.chainLeft + math.sin(self.angle) * -1 * self.chainRight) ##for now, chainLeft and chain Right should be equal
            self.velocity_right = self.translation_Factor(math.cos(self.angle)* self.chainRight + math.sin(self.angle)  * self.chainRight)

        else: # manual mode activated
            self.velocity_right = vel_Right
            self.velocity_left = vel_Left



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
        print("Bots_Bento_Base_Control_v1.0")
        rclpy.spin(main_node)
        main_node.destroy_node()
        rclpy.shutdown()
    if __name__ == '__main__':
        main()
