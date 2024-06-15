import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, Int32
import threading
import time
import math

# Global variables for shared data
manual_control = Twist()
edge_distance = Twist()
imu_data = 0.0
winch_position = Twist()  # Changed to Twist



# Locks for thread safety
manual_control_lock = threading.Lock()
edge_distance_lock = threading.Lock()
imu_data_lock = threading.Lock()
winch_position_lock = threading.Lock()

# Global variables for parameters
manual_mode = False
sync_winch = False
semi_autonomous = False
autonomous = False
width = 640
height = 480
stop = False

class MyComputationNode(Node):
    def __init__(self):
        super().__init__('my_computation_node')

        # Publishers
        self.publisher_gui_status = self.create_publisher(Twist, '/gui_status', 10)
        self.publisher_winch_position = self.create_publisher(Twist, '/winch_position', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_winch = self.create_publisher(Twist, '/winch', 10)

        # Subscribers
        self.subscription_manual_control = self.create_subscription(
            Twist,
            '/manual_control',
            self.manual_control_callback,
            10
        )
        self.subscription_edge_distance = self.create_subscription(
            Twist,
            '/edge_distance',
            self.edge_distance_callback,
            10
        )
        self.subscription_imu_data = self.create_subscription(
            Float32,
            '/imu_data',
            self.imu_data_callback,
            10
        )
        self.subscription_winch_position = self.create_subscription(
            Twist,
            '/winch_position',
            self.winch_position_callback,
            10
        )
        self.sub_base_to_winch = self.create_subscription(Twist, "/base_to_winch", self.callback_base_to_winch, QoSProfile(depth = 10))

        #Variables of Node
        self.chainLeft = 0.0
        self.chainRight = 0.0
        self.angle = 0.0
        self.translation_Factor = 98/47


        # Parameter listeners
        self.param_manual_mode = self.declare_parameter('manual_mode', False).value     #name changed
        self.param_sync_winch = self.declare_parameter('sync_winch', False).value     #name changed
        self.param_semi_autonomous = self.declare_parameter('semi_autonomous', False).value     #name changed
        self.param_autonomous = self.declare_parameter('autonomous', False).value       #name changed


        self.param_width = self.declare_parameter('width', 640).value
        self.param_height = self.declare_parameter('height', 480).value
        self.param_stop = self.declare_parameter('stop', False).value

        self.add_on_set_parameters_callback(self.param_callback)

        # Start the publisher thread
        self.publisher_thread = threading.Thread(target=self.publisher_loop)
        self.publisher_thread.start()
    
    def param_callback(self, params):
        global manual_mode,sync_winch,semi_autonomous, autonomous, width, height, stop
        for param in params:
            if param.name == 'manual_mode':
                manual_mode = param.value
            elif param.name == 'sync_winch':
                sync_winch = param.value
            elif param.name == 'semi_autonomous':
                semi_autonomous = param.value
            elif param.name == 'autonomous':
                autonomous = param.value
            elif param.name == 'width':
                width = param.value
            elif param.name == 'height':
                height = param.value
            elif param.name == 'stop':
                stop = param.value

        return SetParametersResult(successful=True)

    def manual_control_callback(self, msg):
        global manual_control
        with manual_control_lock:
            manual_control = msg

    def edge_distance_callback(self, msg):
        global edge_distance
        with edge_distance_lock:
            edge_distance = msg

    def imu_data_callback(self, msg):
        global imu_data
        with imu_data_lock:
            imu_data = msg.data

    def winch_position_callback(self, msg):
        global winch_position
        with winch_position_lock:
            winch_position = msg

    def callback_base_to_winch(self, msg):

        self.chainLeft = msg.linear.x       #Velocity left chain update
        self.chainRight = msg.linear.y      #Velocity right chain update
    

    def publisher_loop(self):
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            self.publish_gui_status()
            self.publish_winch_position()
            self.publish_cmd_vel()
            self.publish_winch()
            time.sleep(1/25)

    def publish_gui_status(self):
        gui_status_msg = Twist()
        with manual_control_lock:
            gui_status_msg.linear.x = manual_control.linear.x
            gui_status_msg.linear.y = manual_control.linear.y
            gui_status_msg.angular.y = manual_control.angular.y
        self.publisher_gui_status.publish(gui_status_msg)

    def publish_winch_position(self):
        winch_position_msg = Twist()
        with manual_control_lock:
            winch_position_msg.linear.x = manual_control.linear.x
            winch_position_msg.linear.y = manual_control.linear.y
        self.publisher_winch_position.publish(winch_position_msg)

    def publish_cmd_vel(self):
        cmd_vel_msg = Twist()       #The chain-data is transported via the linear part of manual_control
        with manual_control_lock:   
            if stop == False:
                cmd_vel_msg.linear.x = manual_control.linear.x / 2.0
                cmd_vel_msg.linear.y = manual_control.linear.y / 2.0
            
        self.publisher_cmd_vel.publish(cmd_vel_msg)

    def publish_winch(self):
        winch_msg = Twist()         #The winch-data is transported via the angular part of manual_control
        with manual_control_lock:
            temp = imu_data
            self.angle = (temp + 1.0) * math.pi # Angle
            print(f'the imu angle is:  {self.angle}')
            
            print(f'the left winch velocity is:  {winch_msg.linear.x}')
            print(f'the right winch velocity is:  {winch_msg.linear.y}')
            print(f'the left chain velocity is:  {self.chainLeft}')
            print(f'the right chain velocity is:  {self.chainRight}')

            if abs(self.chainLeft - self.chainRight) <= 0.1: ##for now, chainLeft and chain Right should be parallel
                winch_msg.linear.x = self.translation_Factor*(math.cos(self.angle)* self.chainLeft)
                winch_msg.linear.y = -1*self.translation_Factor*(math.cos(self.angle)* self.chainLeft)
            else:
                winch_msg.linear.x = 0.0
                winch_msg.linear.y = 0.0
     
            
            if stop == False:  
                if manual_mode:
                    winch_msg.linear.x = manual_control.angular.x
                    winch_msg.linear.y = manual_control.angular.y
                elif sync_winch:
                    right_left = manual_control.angular.x
                    up_down= manual_control.angular.y
                    if abs(right_left) > 0.01:
                        winch_msg.linear.x = -1.0 * right_left
                        winch_msg.linear.y = 1.0 *right_left
                    elif abs(up_down) > 0.01:
                        winch_msg.linear.x = 1.0 *up_down
                        winch_msg.linear.y = 1.0 *up_down
                    else:
                        winch_msg.linear.x = 0.0
                        winch_msg.linear.y = 0.0
                elif semi_autonomous: 
                    self.angle = (imu_data + 1.0) * math.pi # Angle
                    print(self.angle,"the imu angle is:  %d")
                    if abs(self.chainLeft - self.chainRight) <= 0.01: ##for now, chainLeft and chain Right should be antiparallel ->chainLeft = velocity from now on
                        winch_msg.linear.x = self.translation_Factor*(math.cos(self.angle)* self.chainLeft + math.sin(self.angle) * -1 * self.chainLeft)
                        winch_msg.linear.y = -1*self.translation_Factor*(math.cos(self.angle)* self.chainLeft + math.sin(self.angle)  * self.chainLeft)
                    else:
                        winch_msg.linear.x = 0.0
                        winch_msg.linear.y = 0.0

                elif autonomous:
                    print("not ready")

        self.publisher_winch.publish(winch_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyComputationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
