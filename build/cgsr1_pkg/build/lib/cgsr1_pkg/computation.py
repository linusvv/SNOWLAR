import rclpy
from rclpy.node import Node
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

        # Start the publisher thread
        self.publisher_thread = threading.Thread(target=self.publisher_loop)
        self.publisher_thread.start()

        # Set parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('switch', False),
                ('autonomous', False),
                ('width', 0),
                ('height', 0),
                ('stop', False)
            ]
        )

        # Get parameters
        self.get_logger().info("Waiting for parameters to be set...")
        self.get_parameters([
            'switch',
            'autonomous',
            'width',
            'height',
            'stop'
        ], self.param_callback)

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

    def print_parameter_changes(self, parameters):
        for name, parameter in parameters.items():
            old_value = parameter.old_value
            new_value = parameter.value
            if old_value != new_value:
                self.get_logger().info(f"Parameter '{name}' changed: {old_value} -> {new_value}")


    def param_callback(self, future):
        parameters = future.result()
        global switch, autonomous, width, height, stop
        switch = parameters['switch'].value
        autonomous = parameters['autonomous'].value
        width = parameters['width'].value
        height = parameters['height'].value
        stop = parameters['stop'].value

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
            cmd_vel_msg.linear.x = manual_control.linear.x
            cmd_vel_msg.linear.y = manual_control.linear.y
        self.publisher_cmd_vel.publish(cmd_vel_msg)

    def publish_winch(self):
        winch_msg = Twist()
        with manual_control_lock:
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
