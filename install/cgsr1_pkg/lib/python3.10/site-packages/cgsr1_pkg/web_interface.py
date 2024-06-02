import os
from flask import Flask, request, jsonify, send_file
import rclpy
from geometry_msgs.msg import Twist

# Get the current directory of the script
INDEX_FILE_PATH = os.path.expanduser("~/snowlar_ws_v1/src/cgsr1_pkg/cgsr1_pkg/static/index.html")


app = Flask(__name__)
node = None
publisher = None

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
    twist.linear.y = x
    twist.linear.x = y
    
    if publisher:
        publisher.publish(twist)
        print("Message published")  # Debug print
    else:
        print("Publisher not initialized")  # Debug print
        return jsonify({"status": "error", "message": "Publisher not initialized"}), 500
    
    return jsonify({"status": "success", "x": x, "y": y})

def start_web_interface():
    print("Starting web interface")  # Debug print
    app.run(host='0.0.0.0', port=8080)

def main():
    global node, publisher
    print("Initializing ROS")  # Debug print
    rclpy.init()
    node = rclpy.create_node('web_interface')
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    print("ROS node and publisher initialized")  # Debug print
    
    try:
        start_web_interface()
    except KeyboardInterrupt:
        print("Shutting down")  # Debug print
    finally:
        rclpy.shutdown()
        print("ROS shutdown complete")  # Debug print

if __name__ == '__main__':
    main()

