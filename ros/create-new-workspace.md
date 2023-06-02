# Create New Workspace

REF = [Creating a workspace for catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Create a new workspace called "sample_ws" in the home folder. This assumes ROS has already been installed and the main setup.bash script has already been sourced and added to ~/.bashrc.

```bash
cd ~
mkdir -p sample_ws/src  # Set up basic structure of catkin workspace.
cd ~/sample_ws  # catkin_make must be run from root folder of workspace.
catkin_make  # Complile the workspace. This creates the build and devel folders and src/CMakeLists.txt symlink file.
echo "source /home/[user]/sample_ws/devel/setup.bash" >> ~/.bashrc  # Add setup.bash source command for this specific workspace.
```
