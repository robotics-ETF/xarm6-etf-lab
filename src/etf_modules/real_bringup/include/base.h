#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class BaseNode : public rclcpp::Node
{
public:
    BaseNode(const std::string node_name, const int period, const std::string time_unit = "milliseconds");

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    // rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_publisher;   // Does not work?
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_states_subscription;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client;

    int period;
    int state;
	Eigen::VectorXf joint_states;
	trajectory_msgs::msg::JointTrajectory trajectory;
    
    float max_lin_vel = 500;      // in [mm/s]
    float max_lin_acc = 500;      // in [mm/s²]
    float max_ang_vel = 1.3;      // in [rad/s]
    float max_ang_acc = 1.3;      // in [rad/s²]
    const std::vector<float> home_angles = {0, 0, 0, M_PI, M_PI_2, 0};
    std::vector<float> current_angles;
    std::vector<float> current_pose;

    virtual void baseCallback() = 0;
    void jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
	void publishTrajectory(const std::vector<Eigen::VectorXf> path, const std::vector<float> path_times, float init_time = 0);
    void moveGripper(float position, float max_effort = 5.0);
    
};
