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
automation_control = Twist()

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
brush = False

class MyComputationNode(Node):
    def __init__(self):
        super().__init__('my_computation_node')

        # Publishers
        self.publisher_gui_status = self.create_publisher(Twist, '/gui_status', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel_automated', 10) ###added on 18.06.2024 - not implemented
        self.publisher_winch = self.create_publisher(Twist, '/winch', 10)
        self.publisher_start = self.create_publisher(Twist, '/start_automation', 10)
        self.publisher_brush = self.create_publisher(Float32, 'olive/servo/brush/goal/velocity', QoSProfile(depth=10))

        # Subscribers
        self.subscription_manual_control = self.create_subscription(Twist,'/manual_control',self.manual_control_callback,10)
        self.subscription_edge_distance = self.create_subscription(Twist,'/edge_distance',self.edge_distance_callback,10)
        self.subscription_imu_data = self.create_subscription(Float32,'/imu_data',self.imu_data_callback,10)
        self.subscription_winch_position = self.create_subscription(Twist,'/winch_position',self.winch_position_callback,10)
        self.sub_base_to_winch = self.create_subscription(Twist, "/base_to_winch", self.callback_base_to_winch, QoSProfile(depth=10))
        self.sub_automation = self.create_subscription(Twist, "/automation", self.automation_callback, QoSProfile(depth=10))

        # Subscribers for parameter topics
        self.subscription_manual_mode = self.create_subscription(Bool, 'manual_mode', self.manual_mode_callback, 10)
        self.subscription_sync_winch = self.create_subscription(Bool, 'sync_winch', self.sync_winch_callback, 10)
        self.subscription_semi_autonomous = self.create_subscription(Bool, 'semi_autonomous', self.semi_autonomous_callback, 10)
        self.subscription_autonomous = self.create_subscription(Bool, 'autonomous', self.autonomous_callback, 10)
        self.subscription_width = self.create_subscription(Int32, 'width', self.width_callback, 10)
        self.subscription_height = self.create_subscription(Int32, 'height', self.height_callback, 10)
        self.subscription_stop = self.create_subscription(Bool, 'stop', self.stop_callback, 10)
        self.subscription_brush = self.create_subscription(Bool, 'brush', self.brush_callback, 10)

        # Variables of Node
        self.chainLeft = 0.0
        self.chainRight = 0.0
        self.angle = 0.0
        self.translation_Factor = 98 / 47

        # Start the publisher thread
        self.publisher_thread = threading.Thread(target=self.publisher_loop)
        self.publisher_thread.start()

    # Parameter topic callbacks
    def manual_mode_callback(self, msg):
        global manual_mode
        manual_mode = msg.data
        self.get_logger().info(f'Manual mode updated: {manual_mode}')

    def sync_winch_callback(self, msg):
        global sync_winch
        sync_winch = msg.data
        self.get_logger().info(f'Sync winch updated: {sync_winch}')

    def semi_autonomous_callback(self, msg):
        global semi_autonomous
        semi_autonomous = msg.data
        self.get_logger().info(f'Semi-autonomous mode updated: {semi_autonomous}')

    def autonomous_callback(self, msg):
        global autonomous
        autonomous = msg.data
        self.get_logger().info(f'Autonomous mode updated: {autonomous}')

    def width_callback(self, msg):
        global width
        width = msg.data
        self.get_logger().info(f'Width updated: {width}')

    def height_callback(self, msg):
        global height
        height = msg.data
        self.get_logger().info(f'Height updated: {height}')

    def stop_callback(self, msg):
        global stop
        stop = msg.data
        self.get_logger().info(f'Stop updated: {stop}')
    
    def brush_callback(self, msg):
        global brush
        brush = msg.data
        self.get_logger().info(f'Stop updated: {brush}')

    def manual_control_callback(self, msg):
        global manual_control
        with manual_control_lock:
            manual_control = msg

    # Parameter Callback
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

    def automation_callback(self, msg):
        global automation_control
        automation_control = msg

    def callback_base_to_winch(self, msg):
        self.chainLeft = msg.linear.x  # Velocity left chain update
        self.chainRight = msg.linear.y  # Velocity right chain update



    # Loop Function to publish the relevant data
    def publisher_loop(self):
        rate = 25
        while rclpy.ok():
            self.publish_gui_status()
            self.publish_cmd_vel()
            self.publish_winch()
            self.publish_brush()
            time.sleep(1 / rate)

    ### Publishers ###
    # Publishes current manual control (no idea why the name is so weird) -- did we replace gui status with something else???
    def publish_gui_status(self):
        gui_status_msg = Twist()
        with manual_control_lock:
            gui_status_msg.linear.x = manual_control.linear.x
            gui_status_msg.linear.y = manual_control.linear.y
            gui_status_msg.angular.y = manual_control.angular.y
        self.publisher_gui_status.publish(gui_status_msg)

    # Publishes cmd_vel to base_control
    def publish_cmd_vel(self):
        cmd_vel_msg = Twist()  # The chain-data is transported via the linear part of manual_control
        if not stop:
            if manual_mode or semi_autonomous or sync_winch:
                cmd_vel_msg.linear.x = manual_control.linear.x
                cmd_vel_msg.linear.y = manual_control.linear.y
            if autonomous:
                self.publish_start(1.0)
                cmd_vel_msg.linear.x = automation_control.linear.x
                cmd_vel_msg.linear.y = automation_control.linear.y
        self.publisher_cmd_vel.publish(cmd_vel_msg)

    def publish_start(self, i):
        msg = Twist()
        msg.linear.x = i
        self.publisher_start.publish(msg)


    # Publishes the winch data to winch motion. Calculation for winch is included. If that code is stupid. It's not my fault :)
    def publish_winch(self):
        winch_msg = Twist()  # The winch-data is transported via the angular part of manual_control
        with manual_control_lock:
            temp = imu_data
            self.angle = (temp + 1.0) * math.pi  # Angle
            print(f'manual_mode  {manual_mode}')

            if abs(self.chainLeft - self.chainRight) <= 0.1:  # for now, chainLeft and chain Right should be parallel
                winch_msg.linear.x = -1.0 * self.translation_Factor * (math.cos(self.angle) * self.chainLeft)
                winch_msg.linear.y = self.translation_Factor * (math.cos(self.angle) * self.chainLeft)
            else:
                winch_msg.linear.x = 0.0
                winch_msg.linear.y = 0.0

            if not stop:
                if manual_mode:
                    winch_msg.linear.x = manual_control.angular.x
                    winch_msg.linear.y = manual_control.angular.y
                elif sync_winch:
                    right_left = manual_control.angular.x
                    up_down = manual_control.angular.y
                    if abs(right_left) > 0.01:
                        winch_msg.linear.x = -1.0 * right_left
                        winch_msg.linear.y = 1.0 * right_left
                    elif abs(up_down) > 0.01:
                        winch_msg.linear.x = 1.0 * up_down
                        winch_msg.linear.y = 1.0 * up_down
                    else:
                        winch_msg.linear.x = 0.0
                        winch_msg.linear.y = 0.0
                elif semi_autonomous:
                    self.angle = (imu_data + 1.0) * math.pi  # Angle
                    print(self.angle, "the imu angle is:  %d")
                    if abs(self.chainLeft + self.chainRight) <= 0.1:  # for now, chainLeft and chain Right should be parallel
                        winch_msg.linear.x = -1.0 * self.translation_Factor * (math.cos(self.angle) * self.chainLeft + math.sin(self.angle) * self.chainLeft)
                        winch_msg.linear.y = self.translation_Factor * (math.cos(self.angle) * self.chainLeft + (-1)* math.sin(self.angle) * self.chainLeft)
                    else:
                        winch_msg.linear.x = 0.0
                        winch_msg.linear.y = 0.0
                elif autonomous:
                    print("autonomous mode")
                    self.angle = (imu_data + 1.0) * math.pi  # Angle
                    print(self.angle, "the imu angle is:  %d")
                    if abs(self.chainLeft + self.chainRight) <= 0.1:  # for now, chainLeft and chain Right should be parallel
                        winch_msg.linear.x = -1.0 * self.translation_Factor * (math.cos(self.angle) * self.chainLeft)
                        winch_msg.linear.y = self.translation_Factor * (math.cos(self.angle) * self.chainLeft)
                    else:
                        winch_msg.linear.x = 0.0
                        winch_msg.linear.y = 0.0
        self.publisher_winch.publish(winch_msg)


    #PUBLISHES BRUSH DIRECTLY TO MOTOR
    def publish_brush(self):
        ##DEFINE BRUSH MAX VELOCITY
        brush_max_velocity = 3.0
        msg = Float32()
        if brush == True:
            msg.data = float(brush_max_velocity)
        else:
            msg.data = float(0)
        self.publisher_brush.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = MyComputationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
