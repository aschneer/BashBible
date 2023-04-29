# ROS Cheat Sheet

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