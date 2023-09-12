#include "base/Robot.h"

sim_bringup::Robot::Robot(const std::string config_file_path)
{
    std::string project_abs_path = std::string(__FILE__);
    for (int i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node = YAML::LoadFile(project_abs_path + config_file_path);
    YAML::Node robot_node = node["robot"];

    max_lin_vel = robot_node["max_lin_vel"].as<float>();
    max_lin_acc = robot_node["max_lin_acc"].as<float>();
    max_ang_vel = robot_node["max_ang_vel"].as<float>();
    max_ang_acc = robot_node["max_ang_acc"].as<float>();

    Eigen::VectorXf home_joints_coord(robot_node["home"].size());
    for (int i = 0; i < home_joints_coord.size(); i++)
        home_joints_coord(i) = robot_node["home"][i].as<float>();
    home_joints_state = std::make_shared<base::RealVectorSpaceState>(home_joints_coord);

    joints_state = nullptr;
}

void sim_bringup::Robot::jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
	std::vector<double> positions = msg->actual.positions;
	Eigen::VectorXf q(6);
	q << positions[0], positions[1], positions[2], positions[3], positions[4], positions[5];
    joints_state = std::make_shared<base::RealVectorSpaceState>(q);
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot joint states: (%f, %f, %f, %f, %f, %f).", q(0), q(1), q(2), q(3), q(4), q(5));
}

bool sim_bringup::Robot::isReady()
{
    if (joints_state == nullptr)
        return false;

    return true;
}

bool sim_bringup::Robot::isReached(std::shared_ptr<base::State> q, float tol)
{
    if ((joints_state->getCoord() - q->getCoord()).norm() < tol)
        return true;

    return false;
}