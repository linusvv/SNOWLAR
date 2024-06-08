import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, Int32
import threading

# Global variables for shared data
manual_control = Twist()
edge_distance = Twist()
imu_data = Float32()
winch_position = Twist()  # Changed to Twist

# Locks for thread safety
manual_control_lock = threading.Lock()
edge_distance_lock = threading.Lock()
imu_data_lock = threading.Lock()
winch_position_lock = threading.Lock()

# Global variables for parameters
switch = False
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

        # Parameter listeners
        self.param_switch = self.declare_parameter('switch', False).value
        self.param_autonomous = self.declare_parameter('autonomous', False).value
        self.param_width = self.declare_parameter('width', 640).value
        self.param_height = self.declare_parameter('height', 480).value
        self.param_stop = self.declare_parameter('stop', False).value

        self.add_on_set_parameters_callback(self.param_callback)

        # Start the publisher thread
        self.publisher_thread = threading.Thread(target=self.publisher_loop)
        self.publisher_thread.start()
    
    def param_callback(self, params):
        global switch, autonomous, width, height, stop
        for param in params:
            if param.name == 'switch':
                switch = param.value
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
            imu_data = msg

    def winch_position_callback(self, msg):
        global winch_position
        with winch_position_lock:
            winch_position = msg

    def publisher_loop(self):
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            self.publish_gui_status()
            self.publish_winch_position()
            self.publish_cmd_vel()
            self.publish_winch()
            rate.sleep()

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
        cmd_vel_msg = Twist()
        with manual_control_lock:
            if switch:
                cmd_vel_msg.linear.x = manual_control.linear.x / 2.0
                cmd_vel_msg.linear.y = manual_control.linear.y / 2.0
            else:
                cmd_vel_msg.linear.x = manual_control.linear.x
                cmd_vel_msg.linear.y = manual_control.linear.y
        self.publisher_cmd_vel.publish(cmd_vel_msg)

    def publish_winch(self):
        winch_msg = Twist()
        with manual_control_lock:
            if switch:
                winch_msg.linear.x = manual_control.linear.x / 2.0
                winch_msg.linear.y = manual_control.linear.y / 2.0
            else:
                winch_msg.linear.x = manual_control.angular.x
                winch_msg.linear.y = manual_control.angular.y
        self.publisher_winch.publish(winch_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyComputationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
