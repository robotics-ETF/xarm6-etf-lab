#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TestMoveXarm6Node(Node):

    def __init__(self):
        super().__init__('test_move_xarm6_node')
        publish_topic = '/xarm6_traj_controller/joint_trajectory'
        self.trajectory_publihser = \
            self.create_publisher(JointTrajectory, publish_topic, 10)
        timer_period = 5.0
        self.timer = self.create_timer(timer_period,
                self.timer_callback)
        self.joints = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            ]

    def timer_callback(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        joint_point1 = JointTrajectoryPoint()
        joint_point1.positions = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            ]
        joint_point1.time_from_start = Duration(sec=1)
        trajectory.points.append(joint_point1)

        joint_point2 = JointTrajectoryPoint()
        joint_point2.positions = [
            0.5,
            0.0,
            -0.6,
            0.0,
            0.0,
            0.0,
            ]
        joint_point2.time_from_start = Duration(sec=2)
        trajectory.points.append(joint_point2)

        joint_point3 = JointTrajectoryPoint()
        joint_point3.positions = [
            -0.5,
            0.0,
            -0.6,
            0.0,
            0.0,
            0.0,
            ]
        joint_point3.time_from_start = Duration(sec=3)
        trajectory.points.append(joint_point3)

        self.trajectory_publihser.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    joint_publisher = TestMoveXarm6Node()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
