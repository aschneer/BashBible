# URDF Robot Models

(from Google Bard).

Code is XML.

# Links

links

# Joints

URDF (Unified Robot Description Format) defines six basic types of joints for constructing articulated robot models:

1. **Fixed joint:** This joint represents a rigid connection between two links, allowing no movement between them.
	```xml
	<joint name="fixed_joint" type="fixed">
		<parent link="base_link"/>
		<child link="link1"/>
		<origin xyz="0 0 0" rpy="0 0 0" />
	</joint>
	```
2. **Revolute joint:** This joint represents a rotational joint around a single axis. It allows for one degree of freedom (DOF), enabling rotation about the specified axis.
	```xml
	<joint name="revolute_joint" type="revolute">
		<parent link="base_link"/>
		<child link="link1"/>
		<origin xyz="0 0 0" rpy="0 0 0" />
		<axis xyz="0 0 1"/>
		<limit lower="0" upper="${pi/2}" velocity="100" effort="100"/>
	</joint>
	```
3. **Prismatic joint:** This joint represents a translational joint along a single axis. It allows for one DOF, enabling translation along the specified axis.
	```xml
	<joint name="prismatic_joint" type="prismatic">
		<parent link="base_link"/>
		<child link="link1"/>
		<origin xyz="0 0 0" rpy="0 0 0" />
		<axis xyz="1 0 0"/>
	</joint>
	```
4. **Continuous joint:** This joint is a variant of the revolute joint, allowing for unlimited rotation around the specified axis. It also provides one DOF.
	```xml
	<joint name="continuous_joint" type="continuous">
		<parent link="base_link"/>
		<child link="link1"/>
		<origin xyz="0 0 0" rpy="0 0 0" />
		<axis xyz="0 0 1"/>
	</joint>
	```
5. **Planar joint:** This joint allows for two DOFs, enabling translation in the plane defined by two axes and rotation about the perpendicular axis.
	```xml
	<joint name="planar_joint" type="planar">
		<parent link="base_link"/>
		<child link="link1"/>
		<origin xyz="0 0 0" rpy="0 0 0" />
		<axis xyz="0 1 0"/>
		<axis xyz="0 0 1"/>
	</joint>
	```
6. **Floating joint:** This joint represents a six-DOF joint that allows for unrestricted movement in space. It is typically used to represent the base link of a robot, as it has no fixed reference point.
	```xml
	<joint name="floating_joint" type="floating">
		<parent link="base_link"/>
		<child link="world"/>
		<origin xyz="0 0 0" rpy="0 0 0" />
	</joint>
	```