# jetson_camera_ros2

This package is about publishing camera image from CSI camera attached to Jetson nano for ROS2.

For more information about interfacing with CIS-Camera refer to below link.  
https://github.com/JetsonHacksNano/CSI-Camera

Please note, this project is still in development and does not work correctly.  
Few things need to be fix the issue.  Any help would be much appreciated. 

To build this package, clone this repository under your ros2 workspace.  

```
$ cd ~/ros2_ws/src
$ git clone https://github.com/kyuhyong/jetson_camera_ros2
$ cd ..
$ colcon build --symlink-install
```
If build was successful, and for the first running the node, be sure to source setup.bash under install directory as below.  

```
$ . install/setup.bash
```
To run the node,
```
$ ros2 run jetson_camera_driver jetson_camera_driver_node
```

## Dependency

