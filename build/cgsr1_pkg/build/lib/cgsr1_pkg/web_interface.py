# cgsr1_pkg/flask_server_with_ros.py
import os
from flask import Flask, request, jsonify, send_file
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
from std_msgs.msg import Float32MultiArray, Int32MultiArray

INDEX_FILE_PATH = os.path.expanduser("~/SNOWLAR/src/cgsr1_pkg/cgsr1_pkg/static/index_debug_alt_alt.html")

app = Flask(__name__)
node = None
manual_control_publisher = None
gui_controller_instance = None

@app.route('/')
def index():
    if os.path.exists(INDEX_FILE_PATH):
        return send_file(INDEX_FILE_PATH)
    else:
        return "index.html not found", 404

@app.route('/joystick', methods=['POST'])
def joystick():
    data = request.get_json()
    
    x = data.get('x')
    y = data.get('y')
    if x is None or y is None:
        return jsonify({"status": "error", "message": "Invalid input"}), 400

    twist = Twist()
    
    # Check the switch parameter to determine where to assign the values
    if gui_controller_instance.param_switch:
        twist.angular.x = x + 0.000000001
        twist.angular.y = y + 0.000000001
    else:
        twist.linear.x = x + 0.000000001
        twist.linear.y = y + 0.000000001
    
    if manual_control_publisher:
        manual_control_publisher.publish(twist)
    else:
        return jsonify({"status": "error", "message": "Joystick publisher not initialized"}), 500
    
    return jsonify({"status": "success", "x": x, "y": y})

@app.route('/winch', methods=['POST'])
def winch():
    data = request.get_json()
    
    x = data.get('x')
    y = data.get('y')

    twist = Twist()
    if x is not None:
        twist.angular.x = x + 0.000000001
    if y is not None:
        twist.angular.y = y + 0.000000001
    
    if manual_control_publisher:
        manual_control_publisher.publish(twist)
    else:
        return jsonify({"status": "error", "message": "Winch publisher not initialized"}), 500
    
    return jsonify({"status": "success", "x": x, "y": y})

@app.route('/switch/<switch_name>', methods=['POST'])
def switch(switch_name):
    data = request.get_json()
    value = data.get('value')

    if value is None:
        return jsonify({"status": "error", "message": "Invalid input"}), 400

    if switch_name == 'snap':
        gui_controller_instance.param_switch = value
    elif switch_name == 'hideSliders':
        gui_controller_instance.param_autonomous = value
    else:
        return jsonify({"status": "error", "message": "Invalid switch name"}), 400

    return jsonify({"status": "success", "switch": switch_name, "value": value})

class GUIController(Node):
    def __init__(self):
        super().__init__('gui_controller')
        
        # Initialize with default values
        self.width = 300
        self.height = 150
        self.dot_x = 0
        self.dot_y = 0

        self.subscription_dimensions = self.create_subscription(
            Int32MultiArray,
            '/gui_dimensions',
            self.dimensions_callback,
            10
        )

        self.subscription_position = self.create_subscription(
            Float32MultiArray,
            '/gui_position',
            self.position_callback,
            10
        )

        # Declare parameters
        self.param_switch = self.declare_parameter('switch', False).value
        self.param_autonomous = self.declare_parameter('autonomous', False).value
        self.param_stop = self.declare_parameter('stop', False).value

    def dimensions_callback(self, msg):
        self.width, self.height = msg.data
        self.get_logger().info(f'Received dimensions: width={self.width}, height={self.height}')

    def position_callback(self, msg):
        self.dot_x, self.dot_y = msg.data
        self.get_logger().info(f'Received position: x={self.dot_x}, y={self.dot_y}')

def ros2_thread():
    rclpy.spin(gui_controller_instance)
    gui_controller_instance.destroy_node()
    rclpy.shutdown()

@app.route('/rectangle-data')
def rectangle_data():
    global gui_controller_instance
    
    data = {
        'width': gui_controller_instance.width,
        'height': gui_controller_instance.height,
        'dot_x': gui_controller_instance.dot_x,
        'dot_y': gui_controller_instance.dot_y
    }
    return jsonify(data)

def start_web_interface():
    app.run(host='0.0.0.0', port=8080)

def main():
    global node, manual_control_publisher, gui_controller_instance
    rclpy.init()
    node = rclpy.create_node('web_interface')
    manual_control_publisher = node.create_publisher(Twist, 'manual_control', 10)
    gui_controller_instance = GUIController()
    threading.Thread(target=ros2_thread, daemon=True).start()
    start_web_interface()

if __name__ == '__main__':
    main()
