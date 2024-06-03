import os
from flask import Flask, request, jsonify, send_file
import rclpy
from geometry_msgs.msg import Twist

# Get the current directory of the script
INDEX_FILE_PATH = os.path.expanduser("~/snowlar_ws/snowlar/src/cgsr1_pkg/cgsr1_pkg/static/index_debug.html")

app = Flask(__name__)
node = None
joystick_publisher = None
winch_publisher = None

@app.route('/')
def index():
    if os.path.exists(INDEX_FILE_PATH):
        return send_file(INDEX_FILE_PATH)
    else:
        return "index.html not found", 404

@app.route('/joystick', methods=['POST'])
def joystick():
    data = request.get_json()
    print(f"Received joystick data: {data}")  # Debug print
    
    x = data.get('x')
    y = data.get('y')
    if x is None or y is None:
        return jsonify({"status": "error", "message": "Invalid input"}), 400

    print(f"Publishing Twist message: linear.x={x}, linear.y={y}")  # Debug print
    
    twist = Twist()
    twist.linear.y = x *20 + 0.000000001
    twist.linear.x = y *20 + 0.000000001
    
    if joystick_publisher:
        joystick_publisher.publish(twist)
        print("Joystick message published")  # Debug print
    else:
        print("Joystick publisher not initialized")  # Debug print
        return jsonify({"status": "error", "message": "Joystick publisher not initialized"}), 500
    
    return jsonify({"status": "success", "x": x, "y": y})

@app.route('/winch', methods=['POST'])
def winch():
    data = request.get_json()
    print(f"Received winch data: {data}")  # Debug print
    
    x = data.get('x')
    y = data.get('y')

    twist = Twist()
    if x is not None:
        print(f"Publishing Twist message: linear.x={x}")  # Debug print
        twist.linear.x = x *20 + 0.000000001
    if y is not None:
        print(f"Publishing Twist message: linear.y={y}")  # Debug print
        twist.linear.y = y *20 + 0.000000001
    
    if winch_publisher:
        winch_publisher.publish(twist)
        print("Winch message published")  # Debug print
    else:
        print("Winch publisher not initialized")  # Debug print
        return jsonify({"status": "error", "message": "Winch publisher not initialized"}), 500
    
    return jsonify({"status": "success", "x": x, "y": y})

def start_web_interface():
    print("Starting web interface")  # Debug print
    app.run(host='0.0.0.0', port=8080)

def main():
    global node, joystick_publisher, winch_publisher
    print("Initializing ROS")  # Debug print
    rclpy.init()
    node = rclpy.create_node('web_interface')
    joystick_publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    winch_publisher = node.create_publisher(Twist, 'winch', 10)
    print("ROS node and publishers initialized")  # Debug print
    
    try:
        start_web_interface()
    except KeyboardInterrupt:
        print("Shutting down")  # Debug print
    finally:
        rclpy.shutdown()
        print("ROS shutdown complete")  # Debug print

if __name__ == '__main__':
    main()
