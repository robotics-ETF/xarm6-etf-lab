#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, ETFSA
# All rights reserved.
#
# Author: Dinko Osmankovic <dinko.osmankovic@etf.unsa.ba>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    
    tf_node_world_base = Node(package = "tf2_ros", 
                   executable = "static_transform_publisher",
                   arguments = ["0", "0", "0.5", "0", "0", "1.571", "world", "link_base"]
				)
    
    tf_node_world_camera = Node(package = "tf2_ros", 
                   executable = "static_transform_publisher",
                   arguments = ["1", "-1", "0.5", "0.771", "0", "-1.771", "link_base", "camera_left"]
				)

    # robot moveit gazebo launch
    # xarm_moveit_config/launch/_robot_moveit_gazebo.launch.py
    robot_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('sim_bringup'), 'launch', '_robot_moveit_gazebo.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': '6',
            'robot_type': 'xarm',
            'no_gui_ctrl': 'false',
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
        }.items(),
    )
    
    return LaunchDescription([
        robot_moveit_gazebo_launch,
        tf_node_world_base,
    	tf_node_world_camera
    ])
