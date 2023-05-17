# Node Sample - Talker

This sample is for a talker node that sends messages at a fixed frequency.

File = `talker.cpp` \
Location = `~/catkin_ws/src/talk-listen-pkg/src/`

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle node_handle;
	ros::Publisher publisher = node_handle.advertise<std_msgs::String> ("talker_topic", 1);
	ros::Rate(10 /* Hz */);
	while(ros::ok()) {
		std_msgs::String msg;
		msg.data = "Hello";
		publisher.publish(msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
```