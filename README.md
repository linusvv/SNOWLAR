![Screenshot 2024-06-02 122437](https://github.com/linusvv/snowlar/assets/86794353/94a05a78-176c-4797-a13c-86c67e2ef37b)
# [SNOWLAR](https://sites.google.com/view/snowlar/)
> **Snowlar hilft Eigenheimsbesitzern mit PV-Anlagen, durch automatisierte Schneeentfernung auch im Winter Strom zu generieren.** <br>
> <sub> designed in Germany in Cooperation with [Olive Robotics](https://www.olive-robotics.com/), [MAKERSPACE](https://maker-space.de/), [UnternehmerTUM](https://www.unternehmertum.de/), [Munich School of Engineering](https://www.ed.tum.de/ed/studium/studienangebot/ingenieurwissenschaften-b-sc/) and [Würmwerkstatt](https://sites.google.com/view/wuermwerkstatt) </sub>

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


TODO: 
- pot. enable slow breaking

### Movement Concept:
The movement concept is based on 2 individally controlled xy motion systems. The first one is the rover, which can drive around utilizing its tracks. The other one is the winch mechanism, being able to pull the rover up the roof, but also able to move the wagon at the upper edge of the solar panels left to right on its extrusion by winding in one winch and extending the other. These 2 systems are coupled in software to ensure safe operation and maneuverability of the rover, without if falling off the roof.

### Hardware:
The Robot is build using mainly a combination of Olive Robotics electronics, Maker Beam kits and 3D printed Parts.
The bigger groups of parts are:
1. The main Frame: It is build from 4 motors as a base in the corners that are connected using Maker Beam extrusions.
2. The chains: The chains are 3D printed, connected with pieces of 2mm Wire and get their traction by optionally adhering foam rubber with epoxy to them. They are connected to the frame using 3D printed sprockets. (Detailed build instructions link)
3. The winch: The winch uses 2 motors, connected by extrusions as a base. Attached to this are the 2 3D printed winch reels and deflection roller assemblys. (Detailed build instructions link)
4. The brush: The mount for the brush is constructed by bending sheet metal and the brush itself has a 3D printed body and foam rubber flaps. (Detailed build instructions link)
5. The wagon: The wagon runs on a 3030 aluminum extrusion by using V-rollers and is moved left to right by the winch mechanism. (Detailed build instructions link)

   
