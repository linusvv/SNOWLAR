import os
from flask import Flask, request, jsonify, send_file
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from std_srvs.srv import SetBool

MANUAL_FILE_PATH = os.path.expanduser("~/SNOWLAR/src/cgsr1_pkg/cgsr1_pkg/static/index_debug_alt_alt.html")
HOME_FILE_PATH = os.path.expanduser("~/SNOWLAR/src/cgsr1_pkg/cgsr1_pkg/static/automation.html")
SETTINGS_FILE_PATH = os.path.expanduser("~/SNOWLAR/src/cgsr1_pkg/cgsr1_pkg/static/settings.html")

app = Flask(__name__)
node = None
manual_control_publisher = None
gui_controller_instance = None

@app.route('/')
def index():
    if os.path.exists(HOME_FILE_PATH):
        return send_file(HOME_FILE_PATH)
    else:
        return "index.html not found", 404

@app.route('/home')
def home():
    return send_file(HOME_FILE_PATH)

@app.route('/manual_control')
def manual_control():
    return send_file(MANUAL_FILE_PATH)

@app.route('/settings')
def settings():
    return send_file(SETTINGS_FILE_PATH)

@app.route('/joystick', methods=['POST'])
def joystick():
    data = request.get_json()
    x = data.get('x')
    y = data.get('y')
    if x is None or y is None:
        return jsonify({"status": "error", "message": "Invalid input"}), 400

    twist = Twist()
    if gui_controller_instance.param_manual_mode:
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

    if switch_name == 'manual_mode':
        gui_controller_instance.set_parameter('manual_mode', value)
    elif switch_name == 'sync_mode':
        gui_controller_instance.set_parameter('sync_winch', value)
    elif switch_name == 'semi_autonomous':
        gui_controller_instance.set_parameter('semi_autonomous', value)
    else:
        return jsonify({"status": "error", "message": "Invalid switch name"}), 400

    return jsonify({"status": "success", "switch": switch_name, "value": value})

@app.route('/calibrate', methods=['POST'])
def calibrate():
    start_calibration = request.form.get('start_calibration', 'true').lower() == 'true'
    response = gui_controller_instance.send_request(start_calibration)
    result = {'success': response.success, 'message': response.message}
    return jsonify(result)

@app.route('/stop', methods=['POST'])
def stop():
    stop_val = request.form.get('stop', 'true').lower() == 'true'
    gui_controller_instance.set_parameter('stop', not gui_controller_instance.param_stop if stop_val else False)
    return jsonify({"status": "success", "stop": stop_val})

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

@app.route('/update_settings', methods=['POST'])
def update_settings():
    data = request.get_json()
    width = data.get('width')
    height = data.get('height')

    if width is None or height is None:
        return jsonify({"status": "error", "message": "Invalid input"}), 400

    gui_controller_instance.set_parameter('width', width)
    gui_controller_instance.set_parameter('height', height)

    return jsonify({"status": "success", "width": width, "height": height})

@app.route('/start_automation', methods=['POST'])
def start_automation():
    data = request.get_json()
    automation_status = data.get('status')

    if automation_status is None:
        return jsonify({"status": "error", "message": "Invalid input"}), 400

    gui_controller_instance.set_parameter('autonomous', automation_status)

    return jsonify({"status": "success", "automation_status": automation_status})

class GUIController(Node):
    def __init__(self):
        super().__init__('gui_controller')
        
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

        self.param_manual_mode = self.declare_parameter('manual_mode', False)
        self.param_semi_autonomous = self.declare_parameter('semi_autonomous', False)
        self.param_sync_winch = self.declare_parameter('sync_winch', False)
        self.param_autonomous = self.declare_parameter('autonomous', False)
        self.param_stop = self.declare_parameter('stop', False)

        self.client = self.create_client(SetBool, 'calibrate_motor')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def update_parameters(self):
        self.param_manual_mode = self.get_parameter('manual_mode').get_parameter_value().bool_value
        self.param_semi_autonomous = self.get_parameter('semi_autonomous').get_parameter_value().bool_value
        self.param_sync_winch = self.get_parameter('sync_winch').get_parameter_value().bool_value
        self.param_autonomous = self.get_parameter('autonomous').get_parameter_value().bool_value
        self.param_stop = self.get_parameter('stop').get_parameter_value().bool_value

    def send_request(self, start_calibration):
        req = SetBool.Request()
        req.data = start_calibration
        self.future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_parameter(self, name, value):
        param = Parameter(name, Parameter.Type.BOOL if isinstance(value, bool) else Parameter.Type.INTEGER, value)
        self.set_parameters([param])
        return {'status': 'success'}

    def dimensions_callback(self, msg):
        self.width, self.height = msg.data
        self.get_logger().info(f'Received dimensions: width={self.width}, height={self.height}')
        self.update_parameters()

    def position_callback(self, msg):
        self.dot_x, self.dot_y = msg.data
        self.get_logger().info(f'Received position: x={self.dot_x}, y={self.dot_y}')
        self.update_parameters()

def ros2_thread():
    rclpy.spin(gui_controller_instance)
    gui_controller_instance.destroy_node()
    rclpy.shutdown()

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
