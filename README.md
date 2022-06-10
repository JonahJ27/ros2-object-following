
# ros2-object-following
Uses computer vision to have a simulated car follow a red ball. All simulation done in gazebo and built with ROS2

To use: First create a development workspace and make a directory called follow_diff_drive the src folder. The colcon build the package.
If you don't know how to do this see:
[link](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)


In order to launch the Gazebo world run in a terminal with ros2 sourced:
```
gazebo --verbose <dev_ws_name>/src/follow_diff_drive/worlds/gazebo_ros_follow.world
```
In a new terminal, source ros2, the cd into your dev_ws and source the underlay with:
```
. install/setup.bash
```

Then to run the following script run: 
```
ros2 run follow_diff_drive cv_publisher
```

In order to make this actually interesting, open a third terminal with the same sourcing and run:
```
ros2 run follow_diff_drive teleop_twist
```
```
The t y u keys can be used to control the ball. 
    g h j
    b n m 
The u i o keys control the car.
    m , . 
    k l ;                                                                      
```
The ball is attached to an invisible car so you can control it as such.
