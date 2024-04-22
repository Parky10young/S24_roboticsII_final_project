# S24_roboticsII
ROS2 Workspace for S24 RoboticsII
```
Group 3 Members: 
Christina Cavalluzzo, RIN 662005890
Gabriela Crother-Collado, RIN 662021320
Ziqiao Fang, RIN 661978765
Yunyoung Park, RIN 662054365
```

**Docker**: Open/access a docker container via a terminal (VNC or SSH)
Run a docker container or access (execute) a running docker container, then build your work space.

Install pupil_apriltags inside the docker to detect apriltags. We used tagStandard41h12 for tag type.
```
pip3 install pupil-apriltags
```

## Activate ROS2 environment
Activate ROS2 environment to run ROS software

**Docker**: Open/access a docker container via a terminal (VNC or SSH)
```
cd ~/codes/[team_name]_ws
source install/setup.bash
```

## Launch tracking nodes!

## run the path planning we need to start the mapping node: 
```
ros2 launch yahboomcar_nav map_gmapping_launch.py
```
## save the map using 
```
ros2 launch yahboomcar_nav save_map_launch.py
```
## start the nav2 pack
```
ros2 launch nav2_bringup bringup_launch.py map:=/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/maps/yahboomcar2.yaml
```
## Then start Tracjectory_node
```
ros2 run tracking_control Trajectory Tracjectory_node
```

### Color Detection and Tracking Node
```
ros2 launch tracking_control tracking_color_object_launch.py
```

### Teleoperation node
```
ros2 run tracking_control joy_safety_ctrl
```
### Launch the robot and camera
For the old camera model (astra pro, Robot 1~6)
```
ros2 launch tracking_control car_camera_pro_bringup_launch.py
```
```
ros2 launch tracking_control car_camera_proplus_bringup_launch.py
```

