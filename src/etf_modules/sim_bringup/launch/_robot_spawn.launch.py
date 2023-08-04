#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, ETFSA
# All rights reserved.
#
# Author: Dinko Osmankovic <dinko.osmankovic@etf.unsa.ba>

import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import OpaqueFunction

    
def launch_setup(context, *args, **kwargs):
    report_type = LaunchConfiguration('report_type', default='rich')     # normal, rich, dev (see: https://github.com/xArm-Developer/xarm_ros#report_type-argument)
    dof = LaunchConfiguration('dof', default=6)
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gazebo_ros2_control/GazeboSystem')
    
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
    load_controller = LaunchConfiguration('load_controller', default=True)
    
    camera_left_x = LaunchConfiguration('camera_left_x', default=1.2)
    camera_left_y = LaunchConfiguration('camera_left_y', default=-0.8)
    camera_left_z = LaunchConfiguration('camera_left_z', default=0.6)
    camera_left_R = LaunchConfiguration('camera_left_R', default=0)
    camera_left_P = LaunchConfiguration('camera_left_P', default=0.15)
    camera_left_Y = LaunchConfiguration('camera_left_Y', default=2.8)
    
    camera_right_x = LaunchConfiguration('camera_right_x', default=1.2)
    camera_right_y = LaunchConfiguration('camera_right_y', default=0.8)
    camera_right_z = LaunchConfiguration('camera_right_z', default=0.6)
    camera_right_R = LaunchConfiguration('camera_right_R', default=0)
    camera_right_P = LaunchConfiguration('camera_right_P', default=0.15)
    camera_right_Y = LaunchConfiguration('camera_right_Y', default=3.5)
    
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    # ros2 control params
    # xarm_controller/launch/lib/robot_controller_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lib', 'robot_controller_lib.py'))
    generate_ros2_control_params_temp_file = getattr(mod, 'generate_ros2_control_params_temp_file')
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}{}_controllers.yaml'.format(robot_type.perform(context), dof.perform(context))),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        ros_namespace=LaunchConfiguration('ros_namespace', default='').perform(context),
        update_rate=1000,
    )

    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'robot_description_lib.py'))
    get_xacro_file_content = getattr(mod, 'get_xacro_file_content')
    robot_description = {
        'robot_description': get_xacro_file_content(
            xacro_file=PathJoinSubstitution([FindPackageShare('sim_bringup'), 'urdf', 'xarm_device.urdf.xacro']), 
            arguments={
                'report_type': report_type,
                'dof': dof,
                'prefix': prefix,
                'robot_type': robot_type,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'ros2_control_plugin': ros2_control_plugin,
                'ros2_control_params': ros2_control_params,
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
                'camera_left_x': camera_left_x,
                'camera_left_y': camera_left_y,
                'camera_left_z': camera_left_z,
                'camera_left_R': camera_left_R,
                'camera_left_P': camera_left_P,
                'camera_left_Y': camera_left_Y,
                'camera_right_x': camera_right_x,
            	'camera_right_y': camera_right_y,
            	'camera_right_z': camera_right_z,
            	'camera_right_R': camera_right_R,
            	'camera_right_P': camera_right_P,
            	'camera_right_Y': camera_right_Y,
            }
        ),
    }

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # gazebo launch
    # gazebo_ros/launch/gazebo.launch.py
    xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('sim_bringup'), 'world', 'etflab.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'world': xarm_gazebo_world,
            'server_required': 'true',
            'gui_required': 'true',
            'headless': 'true',
        }.items(),
    )

    # gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', '{}{}'.format(robot_type.perform(context), dof.perform(context)),
            '-x', '0',
            '-y', '0',
            '-z', '0.5',
            '-Y', '1.571',
        ],
    )

    # Load controllers
    controllers = [
        'joint_state_broadcaster',
        '{}{}{}_traj_controller'.format(prefix.perform(context), robot_type.perform(context), dof.perform(context)),
    ]
    if robot_type.perform(context) == 'xarm' and add_gripper.perform(context) in ('True', 'true'):
        controllers.append('{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)))
    load_controllers = []
    if load_controller.perform(context) in ('True', 'true'):
        for controller in controllers:
            load_controllers.append(Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[
                    controller,
                    '--controller-manager', '{}/controller_manager'.format(ros_namespace)
                ],
            ))
                
    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo_spawn_entity_node,
                on_exit=load_controllers,
            )
        ),
        gazebo_launch,
        robot_state_publisher_node,
        gazebo_spawn_entity_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
