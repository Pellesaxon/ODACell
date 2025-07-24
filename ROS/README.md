# MG400 Moveit workspace
TODO: IMPROVE THIS 

## Usefull commands:

### Build ros2 packages

`colcon build --symlink-install`

### Source (Computer specific)

`source Desktop/ODACell/ROS/install/setup.bash`

### Launch ros2 nodes

`ros2 launch mg400_ros2_bringup workcell.launch.py`

### PrecisionHoming Action for laser_test joint
```
target_distance_x: 0.0321
target_distance_y: 0.0288 
tolerance: 0.00002
timeout: 60s
```

`ros2 action send_goal /precision_homing mg400_msgs/action/PrecisionHoming "{target_distance_x: 0.0321, target_distance_y: 0.0288, tolerance: 0.00002,timeout: {sec: 60, nanosec: 0}}" --feedback`
