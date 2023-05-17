# Node Sample - Listener

This sample is for a listener node that listens for messages.

File = `listener.cpp` \
Location = `~/catkin_ws/src/talk-listen-pkg/src/`

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

void callback(const std_msgs::StringConstPtr& msg) {
	std::cout << msg.data << std::endl;
}

int main (int argc, char** argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle node_handle;
	ros::Subscriber subscriber = node_handle.subscribe("talker_topic", 1, &callback);
	ros::spin();
	return 0;
}
```