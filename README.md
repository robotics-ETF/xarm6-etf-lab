# 1. Introduction
This repository contains simulation models, and corresponding motion planning and controlling demos of the robotic manipulator xArm6 from UFACTORY. There are five modules:
- aruco_calibration
- perception_etflab
- real_bringup
- sim_bringup
- RPMPLv2 (https://github.com/roboticsETF/RPMPLv2.git)

The development and test environment are tested on Ubuntu 22.04 + ROS2 Humble.

# 2. How to use
## 2.1 Obtain source code of "xarm6-etf-lab" repository
Choose the location for a target workspace, e.g.:
```
cd ~/
```
DO NOT omit "--recursive"，or the source code of dependent submodules will not be downloaded:
```
git clone https://github.com/roboticsETF/xarm6-etf-lab.git
```

## 2.2 Update "xarm6-etf-lab" repository
```
make submodules
```

## 2.3 Update "xarm_ros2" submodule
```
cd ~/xarm6-etf-lab/src/external_modules/xarm_ros2
git submodule sync
git submodule update --init --remote
```

## 2.4 Install dependencies
```
cd ~/xarm6-etf-lab
rosdep update
make dependencies
```

## 2.5 Build "xarm6-etf-lab" repository
```
make build
```

## 2.6 Source ros2 and gazebo environment
```
source source_dirs.bash
```

# 3. Run the simulation
## 3.1 Launch the robot using RViz2, MoveIt2 and Gazebo
```
cd ~/xarm6-etf-lab
```
First option (RViz2 + Gazebo):
```
ros2 launch sim_bringup xarm6_etflab.launch.py

# or just type:
make sim
```
Second option (MoveIt2 + RViz2 + Gazebo):
```
ros2 launch sim_bringup xarm6_moveit_gazebo.launch.py
```
Third option (MoveIt2 + RViz2):
```
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.236 [add_gripper:=true]
```

## 3.2 Test demo 1 (xarm6 moving)
```
# In the new tab type:
cd ~/xarm6-etf-lab

# In C++
ros2 run sim_bringup test_move_xarm6

# In Python
ros2 run sim_bringup test_move_xarm6.py
```

## 3.3 Test demo 2 (planners from RPMPLv2 library using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run sim_bringup test_planners
```

# 4. Run the realsense cameras
## 4.1 Launch the cameras using RViz2
```
cd ~/xarm6-etf-lab
ros2 launch real_bringup realsense_etflab.launch.py

# or just type:
make cameras
```

# 5. Run the real robot
## 5.1 Launch the robot using RViz2
```
cd ~/xarm6-etf-lab
ros2 launch real_bringup real_xarm6_etflab.launch.py

# or just type:
make real
```

## 5.2 Test demo 1 (xarm6 moving)
```
# In the new tab type:
cd ~/xarm6-etf-lab

# In C++
ros2 run real_bringup test_move_xarm6

# In Python
ros2 run real_bringup test_move_xarm6.py
```

## 5.3 Test demo 2 (pick-and-place operations)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_pick_and_place
```

## 5.4 Test demo 3 (bottle-and-glass operations)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_bottle_and_glass
```

## 5.5 Test demo 4 (pick-and-place operations using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_pick_and_place_using_cameras
```

## 5.6 Test demo 5 (planners from RPMPLv2 library using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_planners
```

