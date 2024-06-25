# Installation

### 1. Install Ubuntu 22.04 LTS minimal Installation on a x86-Device (arm is also possible but not tested)
### 2. Install from Ubuntu-Store: VS-Code*, Plotjuggler*
### 3. Install ROS2:
  - Update and Upgrade your system

        sudo apt update
        sudo apt upgrade


  - Check for UTF-8 (necessary for ROS2)

        locale  # check for UTF-8
        
        sudo apt update && sudo apt install locales
        sudo locale-gen en_US en_US.UTF-8
        sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8
        
        locale  # verify settings
  - Ensure you have the Ubuntu universe repository

        sudo apt install software-properties-common
        sudo add-apt-repository universe
  - Add ROS2 GPG Key

        sudo apt update && sudo apt install curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

  - Add repository to sources list
    
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  - Update and Upgrade your system after setting up the repositories

        sudo apt update
        sudo apt upgrade

  - Install ROS2 Desktop

        sudo apt install ros-humble-desktop

> [!TIP]
> For more information or debugging visit the [ROS2 Documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

### 4. Install further tools and dependencies:
- Git*:
-       sudo apt git-all
- rig_reconfigure*:
-       sudo apt install ros-humble-rig-reconfigure
- rqt*:
-       sudo apt-get install ros-humble-rqt ros-groovy-rqt-humble-plugins
- python3:
-       sudo apt install python3
- apriltag-msgs:
-       sudo apt install ros-humble-apriltag-msgs
- flask:
-       pip install flask






    
All marked (*) Tools are not absolutely necessary but recommended for debugging and visualization. 
    
