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
Choose a location for a target workspace, e.g.:
```
cd ~/
```
and then clone the repository:
```
git clone https://github.com/roboticsETF/xarm6-etf-lab.git
```

## 2.2 Update "xarm6-etf-lab" repository
```
cd ~/xarm6-etf-lab  # Not required if you are cloning through IDE (e.g., VS Code)
make submodules
```

## 2.3 Update "xarm_ros2" submodule
```
cd ~/xarm6-etf-lab/src/external_modules/xarm_ros2
git submodule sync
git submodule update --init --remote
```

## 2.4 Update "gazebo_ros2_control" submodule
```
cd ~/xarm6-etf-lab/src/external_modules/gazebo_ros2_control
git submodule sync
git submodule update --init --remote
```
Optionally, if you want to change the robot initial configuration (all joint angles are set to zero by default), open the file ```~/xarm6-etf-lab/src/external_modules/gazebo_ros2_control/gazebo_ros2_control/src/gazebo_system.cpp```, and find the function ```GazeboSystem::registerJoints```. Then, find the following code line:
```
initial_position = get_initial_value(joint_info.state_interfaces[i]);
```
Instead of that line, add the following code:
```
if (j == 3) 
  initial_position = M_PI;
else if (j == 4)
  initial_position = M_PI_2;
else
  initial_position = get_initial_value(joint_info.state_interfaces[i]);
```
Similarly, you can set any desired angle for the ```j```-th joint.

## 2.5 Install dependencies
```
cd ~/xarm6-etf-lab
rosdep update
make dependencies
```

## 2.6 Build "xarm6-etf-lab" repository
```
make build
```

## 2.7 Source ros2 and gazebo environment
```
source source_dirs.bash
```

# 3. Run the simulation using RViz2, MoveIt2 and Gazebo
First, type:
```
cd ~/xarm6-etf-lab
```
Then, launch the robot using one of the following three options: 

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

## 3.1 Test demo 1 (xarm6 moving)
```
# In the new tab type:
cd ~/xarm6-etf-lab

# In C++
ros2 run sim_bringup test_move_xarm6

# In Python
ros2 run sim_bringup test_move_xarm6.py
```

## 3.2 Test demo 2 (planners from RPMPLv2 library using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run sim_bringup test_planners
```

## 3.3 Test demo 3 (task planning using planners from RPMPLv2 library and using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run sim_bringup test_task_planning
```

## 3.4 Test demo 4 (real-time planning using planners from RPMPLv2 library and using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run sim_bringup test_real_time_planning
```

# 4. Run the realsense cameras using RViz2
First, launch the cameras:
```
cd ~/xarm6-etf-lab
ros2 launch real_bringup realsense_etflab.launch.py

# or just type:
make cameras
```

# 5. Run the real robot using RViz2
First, launch the robot:
```
cd ~/xarm6-etf-lab
ros2 launch real_bringup real_xarm6_etflab.launch.py

# or just type:
make real
```

## 5.1 Test demo 1 (xarm6 moving)
```
# In the new tab type:
cd ~/xarm6-etf-lab

# In C++
ros2 run real_bringup test_move_xarm6

# In Python
ros2 run real_bringup test_move_xarm6.py
```

## 5.2 Test demo 2 (bottle-and-glass operations)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_bottle_and_glass
```

## 5.3 Test demo 3 (pick-and-place operations)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_pick_and_place
```

## 5.4 Test demo 4 (pick-and-place operations using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_pick_and_place_using_cameras
```

## 5.5 Test demo 5 (planners from RPMPLv2 library using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_planners
```

## 5.6 Test demo 6 (task planning using planners from RPMPLv2 library and using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_task_planning
```

## 5.7 Test demo 7 (real-time planning using planners from RPMPLv2 library and using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_real_time_planning
```
