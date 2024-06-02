![Screenshot 2024-06-02 122437](https://github.com/linusvv/snowlar/assets/86794353/94a05a78-176c-4797-a13c-86c67e2ef37b)
# [SNOWLAR](https://sites.google.com/view/snowlar/)
> **Snowlar hilft Eigenheimsbesitzern mit PV-Anlagen, durch automatisierte Schneeentfernung auch im Winter Strom zu generieren.** <br>
> <sub> made in Germany in Cooperation with [Olive Robotics](https://www.olive-robotics.com/), [MAKERSPACE](https://maker-space.de/), [UnternehmerTUM](https://www.unternehmertum.de/) and [Munich School of Engineering](https://www.ed.tum.de/ed/studium/studienangebot/ingenieurwissenschaften-b-sc/) </sub>

<br>
<br>

## Code Repository for the SNOWLAR CGSR1 P5
The Project is based on ROS2, written in Python. Here are the current working nodes:
- **tank_motion** - basecontrol.py
- **joy_steering** - joytobase.py
- **imu_steering** - imutobase.py
- **web_interface** - web_interface.py

And the current test files:
- **test_flask.py**

Web Interface Folder: ../static/
- index.html
- ... tbc

### Run the project:
Initialize Project:
1. Ensure you have installed Ubuntu 22.04 as a VM (make sure you have enough ethernet port forwarding) or locally.
2. Install ROS humble: [ROS2 Docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
3. For setting up the individual Olive components: [Olive Docs](https://www.olive-robotics.com/olive-docs)
4. Create a Network Bridge connecting all the Olive components. For ease of use create a table with the respective part names, part IPs, and part macs. Make sure the ID is set to the same value for each part.
6. Change the code accordingly by replacing part names in the respective nodes.
Execute Code
7. Open the project folder in a terminal. Source ROS with `source snowlar/install/setup.bash`
8. Run the individual ROS2 nodes using `ros2 run <package_name> <node_name>`
9. List active topics `ros2 topic list`
10. Echo topics: `ros2 topic echo <topic_name>`
11. Start the web interface by creating a hotspot. Then connect from any other device to the hotspot and call the IP followed by the port (default: IP:8080). Alternatively, start it locally with `localhost:8080`

