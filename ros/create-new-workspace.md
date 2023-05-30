# Create New Workspace

REF = http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Create a new workspace called "sample_ws" in the home folder.

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
cd ~
mkdir -p sample_ws/src
cd ~/sample_ws
catkin_make
echo "source /home/[user]/sample_ws/devel/setup.bash" >> ~/.bashrc
```
