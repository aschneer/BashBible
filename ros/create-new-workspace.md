# Create New Workspace

REF = http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Create a new workspace called "sample_ws" in the home folder. This assumes ROS has already been installed and the main setup.bash script has already been sourced and added to ~/.bashrc.

```bash
cd ~
mkdir -p sample_ws/src
cd ~/sample_ws
catkin_make
echo "source /home/[user]/sample_ws/devel/setup.bash" >> ~/.bashrc
```
