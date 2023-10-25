# Install ROS

REF = [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

Install ROS Noetic on Ubuntu 20.04.

```bash
# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  # Add ROS deb package endpoint to apt.
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -  # Get git repo key.
sudo apt update
sudo apt install ros-noetic-desktop-full  # Install ROS.
# Source bash setup file for ROS, and set it to source by default in a new terminal.
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc  # Add main setup.bash source command.
source ~/.bashrc  # Source the new line added to .bashrc (otherwise wouldn't execute until bash shell is restarted).
# Install other tools recommended by ROS.
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential  # Install tools for building packages.
sudo rosdep init  # Initialize ROS system dependency tool.
rosdep update  # Update ROS system dependency tool.
# Install Catkin Tools (catkin init, catkin build, wstool, etc.)
# https://catkin-tools.readthedocs.io/en/latest/installing.html
sudo apt-get install wget  # If not already installed
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-catkin-tools  # Catkin tools include catkin init, wstool, catkin build, and others.
```
