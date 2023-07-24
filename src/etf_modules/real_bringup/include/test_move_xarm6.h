#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>

using namespace std::chrono_literals;

class TestMoveXarm6Node : public rclcpp::Node
{
public:
    TestMoveXarm6Node();

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    // rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_publisher;   // Does not work?
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client;
    rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr move_cartesian_client;
	rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_states_subscription;

    int period = 5;
    int state = 0;
    trajectory_msgs::msg::JointTrajectory trajectory;
    std::shared_ptr<rclcpp::Node> move_cartesian_node;

    void goHome();
    void moveGripper(float position, float max_effort = 5.0);
    void move1();
    void moveCartesian(const std::vector<float> &pose, float speed = 10, float acceleration = 10);
    void timerCallback();
    void jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
};
