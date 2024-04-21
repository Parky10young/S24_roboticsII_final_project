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

Install pupil_apriltags inside the docker for Detector
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

## Robot Teleoeration
As the hint suggested, the following function can be done by press and/or hold the key.

- Activate/Deactivate autonomous tracking: press space key
- Moving linear in a direction: press and hold `u    i    o` (left forward, forward, right forward), `j    l` (left right), `m    ,    .` (left backward, backward, right backward)
- Turning counterclockwise/clockwise: `t`/`y`
- Stop the robot: press `k` or anything else

Note that the keyboard teleoperation has higher priority than the autonomous tracking command. In addition, deactivate (press space) the robot if you think the robot is (almost) out of control to protect the robot for potential damages.
