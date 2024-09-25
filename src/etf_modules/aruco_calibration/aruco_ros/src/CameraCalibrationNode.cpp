#include "CameraCalibrationNode.h"

CameraCalibrationNode::CameraCalibrationNode(const std::string &config_file_path) : Node("CameraCalibrationNode")
{
	tf_subscription = this->create_subscription<geometry_msgs::msg::TransformStamped>
		("/aruco_single/transform", 10, std::bind(&CameraCalibrationNode::arucoSingleTFcallback, this, std::placeholders::_1));
	joints_state_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
		("/xarm6_traj_controller/state", 10, std::bind(&CameraCalibrationNode::jointsStateCallback, this, std::placeholders::_1));
	
	std::string project_abs_path(__FILE__);
	for (size_t i = 0; i < 4; i++)
		project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
	std::cout << "Path to URDF: " << project_abs_path + config_file_path << "\n";

    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    robot = std::make_shared<robots::xArm6>(project_abs_path + node["robot"]["urdf"].as<std::string>());

	joints_position = Eigen::VectorXf::Zero(robot->getNumDOFs());
	aruco_bias = Eigen::Vector3f::Zero();
	for (size_t i = 0; i < 3; i++)
		aruco_bias(i) = node["aruco"]["bias"][i].as<float>();
	
}

void CameraCalibrationNode::arucoSingleTFcallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(), "translation");
	RCLCPP_INFO(this->get_logger(), "x: '%f'", msg->transform.translation.x);
	RCLCPP_INFO(this->get_logger(), "y: '%f'", msg->transform.translation.y);
	RCLCPP_INFO(this->get_logger(), "z: '%f'", msg->transform.translation.z);
}

void CameraCalibrationNode::jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    for (size_t i = 0; i < robot->getNumDOFs(); i++)
        joints_position(i) = msg->actual.positions[i];

	frame = robot->computeForwardKinematics(std::make_shared<base::RealVectorSpaceState>(joints_position))->back();
	frame.p += aruco_bias.x() * frame.M.UnitX();
	frame.p += aruco_bias.y() * frame.M.UnitY();
	frame.p += aruco_bias.z() * frame.M.UnitZ();

	// KDL::Rotation rot = frame.M; 	// orijentacija
	// KDL::Vector pos = frame.p;		// pozicija
}

int main(int argc, char *argv[])
{
	const std::string config_file_path = "/aruco_calibration/aruco_ros/data/robot_config.yaml";

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraCalibrationNode>(config_file_path));
	rclcpp::shutdown();
	return 0;
}
