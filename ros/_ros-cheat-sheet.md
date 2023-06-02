# _ROS Cheat Sheet

Command Line Tools

```bash
source [setup file]  # Sets up environment variables for ROS.
	source /opt/ros/noetic/setup.bash
roscore &  # Start ROS Master in background
rosrun [package name] [node/executable]  # Start ROS node
	rosrun turtlesim turtlesim_node
rosnode
	rosnode list  #
	rosnode info  /turtlesim
rostopic
	rostopic list
	rostopic info [topic]
	rostopic echo [topic]  # Prints stream of messages on this topic.
	rostopic hz [topic]  # Shows how frequently topic messages are occurring.
	rostopic pub [topic] [msg type]  # Publish message on a topic once.
		# Since each topic has a specific message type, just [tab] after
		# the topic name and it will fill in the message type.
	rostopic pub [topic] [tab] [tab]  # Publish message on a topic once.
		# First tab fills in message type.
		# Second tab provides blank message for you to fill in.
	rostopic pub [topic] [tab] [tab] -r 1  # Rate (-r) flag, publish 1/sec.
	rostopic pub [topic] [tab] [tab] --once  # Sends 1 message then kills it.
rqt  # All graphical tools
	rqt [tab]  # See all rqt plugins and launch them directly
rospack
	rospack [tab]  # List all rospack options
	rospack list  # List all ROS packages installed
	rospack find  # Search for specific package installed
roscd [pkg name]  # Jump to folder of a package by name
catkin_create_pkg  # Create new package folder
catkin_make  # Compile a workspace

# Autocomplete - How to find commands/tools:
# (might have to hit tab twice)
ros [tab]  # Lists all ROS commands
rosrun [tab]  # Lists available packages to run
rosrun pkg [tab]  # Lists available nodes to run within package "pkg"
catkin [tab]  # Lists all catkin commands
rostopic [tab]  # Tools related to ROS topics

rosbag record -a  # Record all messages to a bag file.
rosbag info [your_bag_file.bag]  # Inspect a bag file.
rosbag play [your_bag_file.bag]  # Play back a bag file.
rosbag filter [your_bag_file.bag] [filtered_bag_file.bag] "topic == '/camera/image' or topic == '/imu/data'"
	# Copy specific messages from one bag file to a new bag file.
# Show message format in bag file:
rosbag info [your_bag_file.bag]  # See list of topics and types within bag file.
rosmsg show [message_type]  # Show format of a message type.
```

```cpp
ros::init(argc, argv, "rosbot_node")  // create ros node
RosbotClass::get_position()  // get published message "position"
ROS_INFO_STREAM("hello")  // print to the console - automatically timestamped
ROS_INFO_STREAM(x_2 << " and " << y_2);
ROS_INFO();
```

Compiling and running

```bash
# Compile using g++
g++ -std=c++11 name.cpp -o name_compiled
# Run
./name_compiled

# Compile using ROS
cd ~/catkin_ws
catkin_make
source devel/setup.bash
# Run
rosrun c_scripts unit1_exercise  # pass package and program
```

Other

```bash
~  # Private namespace prefix
```

See Messages

```bash
# List all available topics
rostopic list
# Get info about a topic
rostopic info /topic_name
# Print messages on a topic
rostopic echo /topic_name
```

Unused

```bash
catkin_init_workspace  # Initialize new workspace (run in xxxxx_ws folder)
catkin build  # Compile a workspace (preferred, not always available)
```
