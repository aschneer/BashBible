# _ROS Cheat Sheet

Setup
```bash
source [setup file]  # Sets up environment variables for ROS.
source /opt/ros/noetic/setup.bash
```

Autocomplete
```bash
# Autocomplete - How to find commands/tools:
# (might have to hit tab twice)
ros [tab]  # Lists all ROS commands
rosrun [tab]  # Lists available packages to run
rosrun pkg [tab]  # Lists available nodes to run within package "pkg"
catkin [tab]  # Lists all catkin commands
rostopic [tab]  # Tools related to ROS topics
```

ROS Master
```bash
roscore
roscore &  # Start ROS Master in background
```

Packages
```bash
rospack [tab]  # List all rospack options
rospack list  # List all ROS packages installed
rospack find  # Search for specific package installed
catkin_create_pkg  # Create new package folder
roscd [pkg name]  # Jump to folder of a package by name
```

Nodes
```bash
rosrun [package name] [node executable]  # Start ROS node
rosrun turtlesim turtlesim_node
rosnode list  # list all nodes currently running
rosnode info /turtlesim
```

Topics
```bash
rostopic list
rostopic info [topic]
rostopic echo [topic]  # Prints stream of messages on this topic.
rostopic hz [topic]  # Shows how frequently topic messages are publishing.
rostopic pub /topic_name message_type "data"  # Publish message on a topic once.
	# Since each topic has a specific message type, just [tab] after
	# the topic name and it will fill in the message type.
rostopic pub /topic_name [tab] [tab]  # Publish message on a topic once.
	# First tab fills in message type.
	# Second tab provides blank message for you to fill in.
rostopic pub /topic_name [tab] [tab] -r 1  # Rate (-r) flag, publish 1/sec.
rostopic pub /topic_name [tab] [tab] --once  # Sends 1 message then kills it.
```

Graphical Tools
```bash
rqt  # All graphical tools
	rqt [tab]  # See all rqt plugins and launch them directly
rviz
```

ROS Bags
```bash
rosbag record -a  # Record all messages to a bag file.
rosbag info [your_bag_file.bag]  # Inspect a bag file.
rosbag play [your_bag_file.bag]  # Play back a bag file.
rosbag filter [your_bag_file.bag] [filtered_bag_file.bag] "topic == '/camera/image' or topic == '/imu/data'"
	# Copy specific messages from one bag file to a new bag file.
rosbag info [your_bag_file.bag]  # See list of topics and types within bag file.
```

ROS Messages
```bash
rosmsg show [message_type]  # Show format of a message type.
```

Catkin Tools (Optional Package)
```bash
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-catkin-tools
sudo pip3 install -U catkin_tools
catkin init  # from workspace root folder
catkin build  # from workspace root folder
```

wstool
[wstool ROS Docs](https://wiki.ros.org/wstool)
```bash
sudo apt-get install python-wstool
sudo pip install -U wstool
wstool init src PATH_TO_ROSINSTALL_FILE.rosinstall  # from workspace root folder
wstool update -t src
```

rosdep
[rosdep ROS Docs](https://wiki.ros.org/rosdep)
```bash
# ROS Noetic
sudo apt-get install python3-rosdep
# ROS Melodic and earlier
sudo apt-get install python-rosdep
# All versions
sudo pip install -U rosdep

sudo rosdep init
rosdep update
rosdep install AMAZING_PACKAGE
rosdep install --from-paths src --ignore-src -r -y
```

```cpp
ros::init(argc, argv, "rosbot_node")  // create ros node
RosbotClass::get_position()  // get published message "position"
ROS_INFO_STREAM("hello")  // print to the console - automatically timestamped
ROS_INFO_STREAM(x_2 << " and " << y_2);
ROS_INFO();
```

Compiling and Running

```bash
# Compile Using ROS - catkin_make
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Compile Using ROS - catkin build
# Catkin Tools must be installed.
cd ~/catkin_ws
catkin build
source devel/setup.bash

# Run
rosrun c_scripts unit1_exercise  # pass package and program

# Compile using g++
g++ -std=c++11 name.cpp -o name_compiled
# Run
./name_compiled
```

Other
```bash
~  # Private namespace prefix
```

Unused
```bash
catkin_init_workspace  # Initialize new workspace (run in xxxxx_ws folder)
```
