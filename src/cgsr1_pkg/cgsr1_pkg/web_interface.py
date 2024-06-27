import os
from flask import Flask, request, jsonify, send_file, Response, stream_with_context
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Float32MultiArray, Int32MultiArray
import time
import json

MANUAL_FILE_PATH = os.path.expanduser("~/Desktop/SNOWLAR/src/cgsr1_pkg/cgsr1_pkg/static/index_debug_alt_alt.html")
HOME_FILE_PATH = os.path.expanduser("~/Desktop/SNOWLAR/src/cgsr1_pkg/cgsr1_pkg/static/automation.html")
SETTINGS_FILE_PATH = os.path.expanduser("~/Desktop/SNOWLAR/src/cgsr1_pkg/cgsr1_pkg/static/settings.html")

app = Flask(__name__)
node = None
manual_control_publisher = None
gui_controller_instance = None

stop_status = False
node_status = False

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
        print('recieved manual mode')
        gui_controller_instance.publish_manual_mode(value)
    elif switch_name == 'sync_mode':
        print('recieved sync mode')
        gui_controller_instance.publish_sync_winch(value)
    elif switch_name == 'semi_autonomous':
        print('recieved auto mode')
        gui_controller_instance.publish_semi_autonomous(value)
    elif switch_name == 'autonomous':
        print('recieved autoauto mode')
        gui_controller_instance.publish_autonomous(value)
    else:
        return jsonify({"status": "error", "message": "Invalid switch name"}), 400

    return jsonify({"status": "success", "switch": switch_name, "value": value})
    
@app.route('/calibrate', methods=['POST'])
def calibrate():
    try:
        response = gui_controller_instance.send_request(True)
        result = {'success': response.success, 'message': 'Calibration started' if response.success else 'Failed to start calibration'}
    except Exception as e:
        result = {'success': False, 'message': str(e)}
    return jsonify(result)

@app.route('/stop', methods=['POST'])
def stop():
    global stop_status
    try:
        stop_val = request.form.get('stop', 'true').lower() == 'true'
        stop_status = stop_val
        gui_controller_instance.publish_stop(stop_val)
        response = {'success': True, 'message': 'Emergency stop activated' if stop_val else 'Emergency stop deactivated'}
    except Exception as e:
        response = {'success': False, 'message': str(e)}
    return jsonify(response)

@app.route('/start_nodes', methods=['POST'])
def start_nodes():
    global node_status
    try:
        activate_nodes = request.form.get('activate_nodes', 'true').lower() == 'true'
        node_status = activate_nodes
        gui_controller_instance.publish_start_nodes(activate_nodes)
        response = {'success': True, 'message': 'Nodes activated' if activate_nodes else 'Nodes deactivated'}
    except Exception as e:
        response = {'success': False, 'message': str(e)}
    return jsonify(response)

@app.route('/rectangle-data')
def rectangle_data():
    global gui_controller_instance  # Ensure we use the global instance
    if gui_controller_instance:
        data = {
            'width': gui_controller_instance.width,
            'height': gui_controller_instance.height,
            'dot_x': gui_controller_instance.dot_x,
            'dot_y': gui_controller_instance.dot_y
        }
        return jsonify(data)
    else:
        return jsonify({"status": "error", "message": "GUIController instance not available"}), 500

@app.route('/update_settings', methods=['POST'])
def update_settings():
    data = request.get_json()
    width = data.get('width')
    height = data.get('height')

    if width is None or height is None:
        return jsonify({"status": "error", "message": "Invalid input"}), 400

    gui_controller_instance.publish_width(width)
    gui_controller_instance.publish_height(height)

    return jsonify({"status": "success", "width": width, "height": height})

@app.route('/start_automation', methods=['POST'])
def start_automation():
    data = request.get_json()
    automation_status = data.get('status')

    if automation_status is None:
        return jsonify({"status": "error", "message": "Invalid input"}), 400

    gui_controller_instance.publish_autonomous(automation_status)

    return jsonify({"status": "success", "automation_status": automation_status})


@app.route('/get_topics', methods=['GET'])
def get_topics():
    def generate():
        while True:
            topics = gui_controller_instance.get_topics()
            yield f"data: {json.dumps(topics)}\n\n"
            time.sleep(1)  # Adjust the interval as needed for real-time updates

    return Response(stream_with_context(generate()), mimetype='text/event-stream')

@app.route('/set_topic_status', methods=['POST'])
def set_topic_status():
    data = request.json
    topic_name = data['topic_name']
    status = data['status']
    gui_controller_instance.set_topic_status(topic_name, status)
    return jsonify({'message': 'Status updated'})

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

        # Publishers for the parameters
        self.publisher_manual_mode = self.create_publisher(Bool, 'manual_mode', 10)
        self.publisher_sync_winch = self.create_publisher(Bool, 'sync_winch', 10)
        self.publisher_semi_autonomous = self.create_publisher(Bool, 'semi_autonomous', 10)
        self.publisher_autonomous = self.create_publisher(Bool, 'autonomous', 10)
        self.publisher_stop = self.create_publisher(Bool, 'stop', 10)
        self.publisher_width = self.create_publisher(Int32, 'width', 10)
        self.publisher_height = self.create_publisher(Int32, 'height', 10)
        self.publisher_start_nodes = self.create_publisher(Bool, 'start_nodes', 10)

        self.client = self.create_client(SetBool, 'calibrate_motor')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.topics_status = []

        self.update_topics_thread = threading.Thread(target=self.update_topics)
        self.update_topics_thread.start()

    def send_request(self, start_calibration):
        req = SetBool.Request()
        req.data = start_calibration
        self.future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def publish_manual_mode(self, value):
        msg = Bool()
        msg.data = value
        self.publisher_manual_mode.publish(msg)

    def publish_sync_winch(self, value):
        msg = Bool()
        msg.data = value
        self.publisher_sync_winch.publish(msg)

    def publish_semi_autonomous(self, value):
        msg = Bool()
        msg.data = value
        self.publisher_semi_autonomous.publish(msg)

    def publish_autonomous(self, value):
        msg = Bool()
        msg.data = value
        self.publisher_autonomous.publish(msg)

    def publish_start_nodes(self, value):
        msg = Bool()
        msg.data = value
        self.publisher_start_nodes.publish(msg)

    def publish_stop(self, value):
        msg = Bool()
        msg.data = value
        self.publisher_stop.publish(msg)

    def publish_width(self, value):
        msg = Int32()
        msg.data = value
        self.publisher_width.publish(msg)

    def publish_height(self, value):
        msg = Int32()
        msg.data = value
        self.publisher_height.publish(msg)

    def dimensions_callback(self, msg):
        self.width, self.height = msg.data
        self.get_logger().info(f'Received dimensions: width={self.width}, height={self.height}')

    def position_callback(self, msg):
        self.dot_x, self.dot_y = msg.data
        self.get_logger().info(f'Received position: dot_x={self.dot_x}, dot_y={self.dot_y}')

    def get_topics(self):
        return self.topics_status
    
    def set_topic_status(self, topic_name, status):
        for topic in self.topics_status:
            if topic['name'] == topic_name:
                topic['active'] = status
                break

    def update_topics(self):
        while True:
            topic_names_and_types = self.get_topic_names_and_types()
            active_topics = set(topic for topic, _ in topic_names_and_types)
            
            self.topics_status = [{'name': topic, 'active': topic in active_topics} for topic in active_topics]
            time.sleep(1)  # Update interval

def ros_thread(gui_controller_instance):
    rclpy.spin(gui_controller_instance)
    gui_controller_instance.destroy_node()
    rclpy.shutdown()

def main(args=None):
    global node, manual_control_publisher, gui_controller_instance  # Ensure we use the global instance
    rclpy.init()
    node = rclpy.create_node('web_interface')
    manual_control_publisher = node.create_publisher(Twist, 'manual_control', 10)
    gui_controller_instance = GUIController()
    threading.Thread(target=ros_thread, args=(gui_controller_instance,), daemon=True).start()
    app.run(host='0.0.0.0', port=5000)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
