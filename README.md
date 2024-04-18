# S24_roboticsII_final_project
Robotics II - Final Project


Group 3 Members: 
Christina Cavalluzzo, RIN 662005890

Gabriela Crother-Collado, RIN 662021320

Ziqiao Fang, RIN 661978765

Yunyoung Park, RIN 662054365


### Launch the robot and camera
**Docker**: Open another terminal and access docker (via VNC or SSH).
For the old camera model (astra pro, Robot 1~6)
```
### Teleoperation node
**Docker**: Open another terminal and access docker (via VNC or SSH). Remeber to **Activate ROS2 environment**. In this node, you can control the robot and activate/deactive tracking.
```
ros2 run joy_safety_ctrl
```

ros2 launch tracking_control car_camera_pro_bringup_launch.py
```
For the new camera model (astra pro plus, Robot 7~). **Unplug and plug** the camera cable if you haven't do so after booting up the robot. The image of the camera cable location is in mini project 2 description.
```
ros2 launch tracking_control car_camera_proplus_bringup_launch.py
```
