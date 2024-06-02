#!/bin/bash
# scripts/start_web_interface.sh

# Start ROSBridge server
roslaunch rosbridge_server rosbridge_websocket.launch &
ROSBRIDGE_PID=$!

# Start Flask web server
python $(rospack find my_python_pkg)/my_python_pkg/web_interface.py &
FLASK_PID=$!

# Wait for processes to finish
wait $ROSBRIDGE_PID
wait $FLASK_PID

