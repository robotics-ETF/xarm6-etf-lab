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
In ```apps/object_segmentation.cpp``` file within ```perception_etflab``` library, you can set ```config_file_path``` to be:
1. ```/perception_etflab/data/sim_perception_etflab_config.yaml``` - Obstacles are random and dynamic, and the law of their motion is defined within ```src/environments/Obstacles.cpp``` file. Point cloud representing obstacles is generated within the code, and then corresponding obstacles are published at a rate of ```1/period``` [Hz] to ```objects_cloud``` topic. All configuration settings (including ```period```) can be set in the used yaml file within ```random_obstacles``` node. 
2. ```/perception_etflab/data/real_perception_etflab_config.yaml``` - Obstacles are static, and they are defined within ```world/etflab.world``` file in ```sim_bringup``` library. After combining point clouds from two cameras (left and right one), a combined point cloud is read from ```pointcloud_combined``` topic. Then, obstacles are published at a rate of cca. 5 [Hz] to ```objects_cloud``` topic after their segmentation is done.

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

Note: For each test file from ```apps``` folder, there is a corresponding yaml file withing ```data``` folder, where all necessary configurations can be set.

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

https://github.com/user-attachments/assets/d47a7aae-8bd0-4d07-8e0b-74627740194f

## 3.3 Test demo 3 (task planning using planners from RPMPLv2 library and using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run sim_bringup test_task_planning
```

## 3.4 Test demo 4 (real-time planning using DRGBT planner from RPMPLv2 library and using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run sim_bringup test_real_time_planning
```

https://github.com/user-attachments/assets/bbbb9d0d-48d0-4280-9f06-73b950a28b6c

https://github.com/user-attachments/assets/1dbe400c-6fd7-4ac3-8c03-2cfc0e70e3fa

## 3.5 Test demo 5 (real-time task planning using DRGBT planner from RPMPLv2 library and using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run sim_bringup test_real_time_task_planning
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
In ```apps/object_segmentation.cpp``` file within ```perception_etflab``` library, ```config_file_path``` must be set to ```/perception_etflab/data/real_perception_etflab_config.yaml```. Two cameras scan the environment, and after combining point clouds from the cameras (left and right one), a combined point cloud is read from ```pointcloud_combined``` topic. Then, obstacles are published at a rate of cca. 25 [Hz] to ```objects_cloud``` topic after their segmentation is done.

Note: For each test file from ```apps``` folder, there is a corresponding yaml file withing ```data``` folder, where all necessary configurations can be set.

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

https://github.com/user-attachments/assets/70666ac8-b1af-4938-9666-510000e30901

## 5.2 Test demo 2 (bottle-and-glass operations)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_bottle_and_glass
```

https://github.com/user-attachments/assets/9540c4eb-da71-41bb-b4e8-047dc8c5b141

## 5.3 Test demo 3 (pick-and-place operations)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_pick_and_place
```

https://github.com/user-attachments/assets/405f5b25-d6c9-4efc-bc87-a82252d1d834

## 5.4 Test demo 4 (pick-and-place operations using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_pick_and_place_using_cameras
```

https://github.com/user-attachments/assets/71545070-5da2-4dde-bb79-0009f89c78ba

## 5.5 Test demo 5 (planners from RPMPLv2 library using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_planners
```

https://github.com/user-attachments/assets/31c04b23-9b33-44bb-94c2-37a41180de98

## 5.6 Test demo 6 (task planning using planners from RPMPLv2 library and using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_task_planning
```

https://github.com/user-attachments/assets/6d78350e-9e64-4400-be94-df87fc070c46

## 5.7 Test demo 7 (real-time planning using DRGBT planner from RPMPLv2 library and using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_real_time_planning
```

https://github.com/user-attachments/assets/9c582143-1d74-496e-a4a0-dde80655efac

https://github.com/user-attachments/assets/1f5303ce-745e-4067-83e6-dcaecc1907e6

## 5.8 Test demo 8 (real-time task planning using DRGBT planner from RPMPLv2 library and using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_real_time_task_planning
```

https://github.com/user-attachments/assets/08de02dd-e1c8-4726-b6c3-f6f8295470d8

https://github.com/user-attachments/assets/68841cae-02b0-4555-a79d-39a97862ca06

https://github.com/user-attachments/assets/f47aad66-ca05-42e0-9328-9a03e8dceee1
