#include "base/BaseNode.h"

real_bringup::BaseNode::BaseNode(const std::string node_name, const std::string config_file_path) : 
    Node(node_name),
    Robot(config_file_path),
    Trajectory(Robot::getMaxAngVel())
{
    project_abs_path = std::string(__FILE__);
    for (int i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node = YAML::LoadFile(project_abs_path + config_file_path);

    period = node["period"].as<int>();    
    timer = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&BaseNode::baseCallback, this));

    Robot::joints_state_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
        ("/xarm6_traj_controller/state", 10, std::bind(&Robot::jointsStateCallback, this, std::placeholders::_1));
    Robot::gripper_node = std::make_shared<rclcpp::Node>("gripper_node");
    Robot::gripper_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>
        (gripper_node, "/xarm_gripper/gripper_action");
    
    Trajectory::publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>
        ("/xarm6_traj_controller/joint_trajectory", 10);

}
