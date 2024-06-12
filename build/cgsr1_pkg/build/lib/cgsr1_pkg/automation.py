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


        #Global varibles:
        self.velocity_x = 0  #actually means turning 
        self.velocity_y = 0  #actuall means driving forward
        self.current_x = 0   #position in x direction
        self.current_y = 0   # position in y direction
        self.max_X = 100     #width of roof
        self.max_Y = 150    #length of roof
        self.helper = 0     
        self.movement_factor = 0.5      # the real x-Axis movement in cm, which the rover does when allowed to move for one time intervall into a specific direction
        self.angle = 0      #current angle of the rover with respect to the roof
        
        # Publisher
        self.pub_automation = self.create_publisher(Twist, "/automation", QoSProfile(depth=10))

        #Subsciber
        self.sub_winch = self.create_subscription(Twist, "/winch", self.callback_winch, QoSProfile(depth=10))
        
        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25
       
        self.thread_main.start()

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:

                while self.current_x < self.max_X:          # while not reached end of x-direction
                    while self.current_y < self.max_Y:      #while not reached end of y-direction
                        self.velocity_x = 0                 #move down the y direction
                        self.velocity_y = 1
                        self.current_y = self.current_y + self.movement_factor

                        # Publish the automation data
                        self.publish_automation(self.pub_automation, self.velocity_x, self.velocity_y)

                        time.sleep(1 / self.rate_control_hz)

                    while self.current_y > 0:               #reached end, we drive back up
                        self.velocity_x = 0
                        self.velocity_y = -1
                        self.current_y = self.current_y - self.movement_factor

                        # Publish the automation data
                        self.publish_automation(self.pub_automation, self.velocity_x, self.velocity_y)

                        time.sleep(1 / self.rate_control_hz)
                    while self.angle < (math.pi *3/2):      #turning by 90 degrees
                        self.velocity_x = 1
                        self.velocity_y = 0

                        # Publish the automation data
                        self.publish_automation(self.pub_automation, self.velocity_x, self.velocity_y)

                        time.sleep(1 / self.rate_control_hz)
                    while self.helper < 30:                 #drives in x direction by 30 cm (at least we hope)
                        self.velocity_x = 0
                        self.velocity_y = 1
                        self.helper = self.helper + self.movement_factor
                        self.current_x = self.current_x + self.movement_factor
                        self.publish_automation(self.pub_automation, self.velocity_x, self.velocity_y)
                        time.sleep(1 / self.rate_control_hz)
                    self.helper = 0
                    while self.angle > math.pi:      #turning back by 90 degrees
                        self.velocity_x = -1
                        self.velocity_y = 0

                        # Publish the automation data
                        self.publish_automation(self.pub_automation, self.velocity_x, self.velocity_y)

                        time.sleep(1 / self.rate_control_hz)

                    time.sleep(1 / self.rate_control_hz)

                self.velocity_x = 0
                self.velocity_y = 0
                self.publish_automation(self.pub_automation, self.velocity_x, self.velocity_y)
            
            
            

    

    def publish_automation(self, publisher, vx,vy):
        automation_msg = Twist()
        automation_msg.linear.x = vx
        automation_msg.linear.y = vy
        automation_msg.angular.x = 0
        automation_msg.angular.y = 0
        

        self.publisher.publish(automation_msg)



    def callback_winch(self, msg):
        vel_Left = msg.angular.x    # manual mode left
        vel_Right = msg.angular.y   # manual mode right
        tempAngle = (msg.angular.z + 1) * math.pi # Angle

        self.angle = tempAngle



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
