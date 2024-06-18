import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import threading
import time
import math

class MainNode(Node):
    def __init__(self, config_path, node_name):
        super().__init__(node_name)


        #Global varibles:

        self.start = 0.0      #says if automation should start or not

        self.velocity_x = 0.0  #actually means turning 
        self.velocity_y = 0.0  #actuall means driving forward
        self.current_x = 0.0   #position in x direction
        self.current_y = 0.0  # position in y direction
        self.max_X = 100.0     #width of roof
        self.max_Y = 150.0   #length of roof
        self.helper = 0.0     
        self.movement_factor = 0.2      # 5 cm/s movement in an direction in 1/rate seconds, in cm
        self.angle = 0.0      #current angle of the rover with respect to the roof

        self.driveSpeed = 0.4
        self.rotationSpeed = 0.8

        self.target_angle = 0.0 # always in the -1 to 1 system!!!
        self.straight_x_vel = 0.0
        self.straight_y_vel = 0.0


        
        # Publisher
        self.pub_automation = self.create_publisher(Twist, "/automation", QoSProfile(depth=10))
        self.pub_automation = self.create_publisher(Twist, "/cmd_vel_straight", QoSProfile(depth=10))

        #Subsciber
        self.sub_imu_data = self.create_subscription(Float32, "/imu_data", self.callback_imu_data, QoSProfile(depth=10))
        self.sub_cmd_vel_straight = self.create_subscription(Twist, "/cmd_vel_automated", self.callback_vel_automated, QoSProfile(depth=10))
        self.subscription_autonomous = self.create_subscription(Bool, 'autonomous', self.autonomous_callback, 10)
        


        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25.0
       
        self.thread_main.start()

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:
            if self.start == 0:
                 time.sleep(1/self.rate_control_hz)

            else:
                while self.current_x < self.max_X and self.start:          # while not reached end of x-direction
                    while self.current_y < self.max_Y and self.start:      #while not reached end of y-direction
                        self.publish_drive_straight((-1.0)*self.driveSpeed, 0.0)
                        self.velocity_x = self.straight_x_vel                 #move down the y direction
                        self.velocity_y = self.straight_y_vel

                        self.current_y = self.current_y + self.movement_factor

                        # Publish the automation data
                        self.publish_automation(self.pub_automation, self.velocity_x, self.velocity_y)

                        time.sleep(1 / self.rate_control_hz)

                    while self.current_y > 0.0 and self.start:               #reached end, we drive back up
                        self.publish_drive_straight(self.driveSpeed, 0.0)
                        self.velocity_x = self.straight_x_vel
                        self.velocity_y = self.straight_y_vel
                        self.current_y = self.current_y - self.movement_factor

                        # Publish the automation data
                        self.publish_automation( self.velocity_x, self.velocity_y)

                        time.sleep(1 / self.rate_control_hz)
                    while self.angle > (-0.5) and self.start:      #turning by 90 degrees TODO fix angle
                        self.velocity_x = 0.0
                        self.velocity_y = -1.0*self.rotationSpeed

                        # Publish the automation data
                        self.publish_automation( self.velocity_x, self.velocity_y)

                        time.sleep(1 / self.rate_control_hz)
                    while self.helper < self.movement_factor*25.0*6.0 and self.start:   #drives in x direction by 30 cm (at least we hope)
                        self.publish_drive_straight(-1*self.driveSpeed,-0.5)
                        self.velocity_x = self.straight_x_vel
                        self.velocity_y = self.straight_y_vel
                        self.helper = self.helper + self.movement_factor
                        self.current_x = self.current_x + self.movement_factor  #update current x value
                        self.publish_automation( self.velocity_x, self.velocity_y)
                        time.sleep(1 / self.rate_control_hz)
                    self.helper = 0.0
                    while self.angle < 0 and self.start:    #turning back by 90 degrees TODO fix angle
                        self.velocity_x = 0.0
                        self.velocity_y = 1*self.rotationSpeed

                        # Publish the automation data
                        self.publish_automation( self.velocity_x, self.velocity_y)

                        time.sleep(1 / self.rate_control_hz)

                    time.sleep(1 / self.rate_control_hz)

            self.velocity_x = 0.0
            self.velocity_y = 0.0
            print("Path ended! Hurray :)")

            self.publish_automation( self.velocity_x, self.velocity_y)
        
            
            
    def publish_drive_straight(self,vx,angle):
        straight_msg = Twist()
        straight_msg.linear.x = vx
        straight_msg.linear.y = angle

        self.publish_drive_straight(straight_msg)
    

    def publish_automation(self, vx,vy):
        automation_msg = Twist()
        automation_msg.linear.x = vx
        automation_msg.linear.y = vy
        automation_msg.angular.x = 0.0
        automation_msg.angular.y = 0.0


        self.pub_automation.publish(automation_msg)

    def callback_vel_automated(self, msg):
         self.straight_x_vel = msg.linear.x
         self.straight_y_vel = msg.linear.y

    def callback_imu_data(self, msg):
        self.angle = msg.data

    def autonomous_callback(self, msg):
        self.start = msg.data



def main(args=None):
        rclpy.init(args=args)
        main_node = MainNode("config/path", "main_node")
        print("whinch control active")
        rclpy.spin(main_node)
        main_node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
        main()
