# ROS-TF-Introduction
A ROS project introducing to the ROS framework 'tf' for handling transforms, by applying the following relationship:
1. The transform from the 'base' coordinate frame to the 'object' coordinate frame consists of a rotation expressed as (roll, pitch, yaw) of (0.79, 0.0, 0.79) followed by a translation of 1.0m along the resulting y-axis and 1.0m along the resulting z-axis. 
2. The transform from the 'base' coordinate frame to the 'robot' coordinate frame consists of a rotation around the z-axis by 1.5 radians followed by a translation along the resulting y-axis of -1.0m. 
3. The transform from the 'robot' coordinate frame to the 'camera' coordinate frame must be defined as follows:
  * The translation component of this transform is (0.0, 0.1, 0.1)
  * The rotation component of this transform must be set such that the camera is pointing directly at the object. In other words, the x-axis of the 'camera' coordinate frame must be pointing directly at the origin of the 'object' coordinate frame.

## Getting Started
Download the folders and add them to your catkin folder.

```
Start your roscore: roscore
In another terminal starts the two_int_talker: rosrun marker_publisher marker_publisher.cpp
In another terminal starts the sum_publisher: rosrun camera_robot camera_robot.py
Then you can visualize the robot with rviz: rosrun rviz rviz
Then select in Fixed Frame "base_frame". Click Add and on the popup screen select the tab "By topic". Select the topic /visualization_marker>Marker. You should be able to see the block, cylinder and arrow. You can also add the item "TF" if you want to see a visual representation of the frames.
```

### Prerequisites
You need a complete installation of ROS.

### Results
![Image of a running example](/execution.png)

## Author
* **Mickaël Trezzy**
