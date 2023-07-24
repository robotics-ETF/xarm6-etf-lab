#include "test_move_xarm6.h"

TestMoveXarm6Node::TestMoveXarm6Node() : Node("test_move_xarm6_node")
{
    trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
    timer = this->create_wall_timer(15s, std::bind(&TestMoveXarm6Node::testPlannersCallback, this));
}

void TestMoveXarm6Node::testPlannersCallback()
{
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    trajectory_msgs::msg::JointTrajectoryPoint point;

    point.positions = {0, 0, 0, 0, 0, 0};
    point.time_from_start.sec = 0;
    trajectory.points.emplace_back(point);
    
    point.positions = {M_PI_2, 0, 0, 0, 0, 0};
    point.time_from_start.sec = 5;
    trajectory.points.emplace_back(point);

    point.positions = {0, 0, 0, 0, 0, 0};
    point.time_from_start.sec = 10;
    trajectory.points.emplace_back(point);

    std::cout << "Publishing trajectory ...\n";
    trajectory_publisher->publish(trajectory);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestMoveXarm6Node>());
    rclcpp::shutdown();
    return 0;
}