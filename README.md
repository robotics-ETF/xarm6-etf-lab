# 1. Introduction
This repository contains simulation models, and corresponding motion planning and controlling demos of the robotic manipulator xArm6 from UFACTORY. There are five modules:
- aruco_calibration
- perception_etflab
- real_bringup
- sim_bringup
- RPMPLv2 (https://github.com/roboticsETF/RPMPLv2.git)

The development and test environment are tested on Ubuntu 22.04 + ROS2 Humble.

The paper describing the implementation details of the proposed real-time DRGBT algorithm is available at [link1-to-paper](https://www.researchgate.net/publication/394461144_Real-Time_Sampling-Based_Safe_Motion_Planning_for_Robotic_Manipulators_in_Dynamic_Environments) or [link2-to-paper](https://ieeexplore.ieee.org/document/11122893).


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
Please upload the file [xarm_user_params.yaml](https://github.com/user-attachments/files/24239995/xarm_user_params.yaml) to the folder ```xarm6-etf-lab/src/external_modules/xarm_ros2/xarm_api/config```.

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
Then, launch the robot using one of the following four options: 

### 1. option (RViz2 + Gazebo):
```
ros2 launch sim_bringup static_xarm6_etflab.launch.py

# or just type:
make sim_static
```
Obstacles are static, and they are defined within ```world/etflab.world``` file in ```sim_bringup``` library (e.g., see example in ```world/etflab1.world```). After combining point clouds from two cameras (left and right one), a combined point cloud is read from ```pointcloud_combined``` topic. Then, obstacles are published at a rate of cca. 5 [Hz] to ```objects_cloud``` topic after their segmentation is done. All configuration settings can be set within ```perception``` node in the yaml file ```/perception_etflab/data/real_perception_etflab_config.yaml``` from ```perception_etflab``` library. Use this option if you want to simulate readings from cameras with their uncertainties.

### 2. option (RViz2 + Gazebo):
```
ros2 launch sim_bringup dynamic_xarm6_etflab.launch.py

# or just type:
make sim_dynamic
```
Obstacles are randomly generated and they are dynamic (they move in robot's workspace). The law of their motion is defined within ```perception_etflab/src/environments/Obstacles.cpp``` file. Point cloud representing obstacles is generated within the code, and then corresponding obstacles are published at a rate of ```1/period``` [Hz] to ```objects_cloud``` topic. All configuration settings (including ```period```) can be set within ```perception``` and ```random_obstacles``` nodes in the yaml file ```/perception_etflab/data/sim_perception_etflab_config.yaml``` from ```perception_etflab``` library. 

Note that if there are defined static obstacles in ```world/etflab.world``` file from ```sim_bringup``` library, they will be shown, but NOT considered during the planning!!! This option does not simulate readings from cameras, yet assume that all obstacles' info are already read.

### 3. option (MoveIt2 + RViz2 + Gazebo):
```
ros2 launch sim_bringup xarm6_moveit_gazebo.launch.py
```

### 4. option (MoveIt2 + RViz2):
```
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.236 [add_gripper:=true]
```

Note: For each test file from ```apps``` folder, there is a corresponding yaml file withing ```data``` folder, where all necessary configurations can be set.

## 3.1 Test xarm6 moving
```
# In the new tab type:
cd ~/xarm6-etf-lab

# In C++
ros2 run sim_bringup test_move_xarm6

# In Python
ros2 run sim_bringup test_move_xarm6.py
```

## 3.2 Test planners from RPMPLv2 library while using cameras
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run sim_bringup test_planners
```

https://github.com/user-attachments/assets/d47a7aae-8bd0-4d07-8e0b-74627740194f

## 3.3 Test task planning using planners from RPMPLv2 library while using cameras
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run sim_bringup test_task_planning
```

## 3.4 Test real-time planning using DRGBT or RRTx planner from RPMPLv2 library while using cameras
```
# In the new tab type:
cd ~/xarm6-etf-lab
# DRGBT planner:
ros2 run sim_bringup test_dynamic_planning
# RRTx planner:
ros2 run sim_bringup test_dynamic_planning2
```

https://github.com/user-attachments/assets/bbbb9d0d-48d0-4280-9f06-73b950a28b6c

https://github.com/user-attachments/assets/1dbe400c-6fd7-4ac3-8c03-2cfc0e70e3fa

## 3.5 Test real-time task planning using DRGBT planner from RPMPLv2 library while using cameras
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run sim_bringup test_dynamic_task_planning
```

For more details about these senarios and for comparison of DRGBT with RRTx and MARS methods, see [video](https://www.youtube.com/watch?v=lG7q0PuFhG0).


# 4. Run the realsense cameras
First, launch the cameras:
```
cd ~/xarm6-etf-lab
ros2 launch real_bringup realsense_etflab.launch.py

# or just type:
make cameras
```

Two cameras scan the environment, and after combining point clouds from the cameras (left and right one), a combined point cloud is read from ```pointcloud_combined``` topic. Then, obstacles are published at a rate of cca. 25 [Hz] to ```objects_cloud``` topic after their segmentation is done. These topic are created in ```perception_etflab``` library.


# 5. Run the real robot in static environments
Note: For each test file in the sequel (which is located in ```apps``` folder), there is a corresponding yaml file withing ```data``` folder, where all necessary configurations can be set.

First, launch the robot:
```
cd ~/xarm6-etf-lab
ros2 launch real_bringup real_xarm6_etflab.launch.py

# or just type:
make real
```

## 5.1 Test xarm6 moving
```
# In the new tab type:
cd ~/xarm6-etf-lab

# In C++
ros2 run real_bringup test_move_xarm6

# In Python
ros2 run real_bringup test_move_xarm6.py
```

Within the function ```real_bringup::MoveXArm6Node::moveXArm6Callback()```, you can test different robot modes as explained [here](https://github.com/xArm-Developer/xarm_ros#report_type-argument) using ```xarm_api``` from [here](https://github.com/xArm-Developer/xarm_ros2/tree/humble).

https://github.com/user-attachments/assets/70666ac8-b1af-4938-9666-510000e30901

## 5.2 Test demo 1 (bottle-and-glass operations)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_demo1
```

https://github.com/user-attachments/assets/9540c4eb-da71-41bb-b4e8-047dc8c5b141

## 5.3 Test demo 2 (pick-and-place operations)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_demo2
```

https://github.com/user-attachments/assets/405f5b25-d6c9-4efc-bc87-a82252d1d834

## 5.4 Test demo 3 (pick-and-place operations using cameras)
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_demo3
```

https://github.com/user-attachments/assets/71545070-5da2-4dde-bb79-0009f89c78ba

## 5.5 Test planners from RPMPLv2 library while using cameras
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_planners
```

https://github.com/user-attachments/assets/31c04b23-9b33-44bb-94c2-37a41180de98

## 5.6 Test task planning using planners from RPMPLv2 library while using cameras
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_task_planning
```

https://github.com/user-attachments/assets/6d78350e-9e64-4400-be94-df87fc070c46


# 6. Run the real robot in dynamic environments
First, open the file ```xarm6-etf-lab/src/external_modules/xarm_ros2/xarm_controller/config/xarm6_controllers.yaml```. It is recommended to set ```controller_manager/ros__parameters/update_rate``` to 500 [Hz] and ```xarm6_traj_controller/ros__parameters/state_publish_rate``` to 250 [Hz].

Note: For each test file in the sequel (which is located in ```apps``` folder), there is a corresponding yaml file withing ```data``` folder, where all necessary configurations can be set.

Now, launch the robot:
```
cd ~/xarm6-etf-lab
ros2 launch real_bringup real_xarm6_etflab.launch.py

# or just type:
make real
```

## 6.1 Test real-time planning using DRGBT planner from RPMPLv2 library while using cameras
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_dynamic_planning
```

https://github.com/user-attachments/assets/9c582143-1d74-496e-a4a0-dde80655efac

https://github.com/user-attachments/assets/1f5303ce-745e-4067-83e6-dcaecc1907e6

## 6.2 Test real-time task planning using DRGBT planner from RPMPLv2 library while using cameras
```
# In the new tab type:
cd ~/xarm6-etf-lab
ros2 run real_bringup test_dynamic_task_planning
```

https://github.com/user-attachments/assets/08de02dd-e1c8-4726-b6c3-f6f8295470d8

https://github.com/user-attachments/assets/68841cae-02b0-4555-a79d-39a97862ca06

https://github.com/user-attachments/assets/f47aad66-ca05-42e0-9328-9a03e8dceee1


For more details about these senarios see [video](https://www.youtube.com/watch?v=lG7q0PuFhG0).
