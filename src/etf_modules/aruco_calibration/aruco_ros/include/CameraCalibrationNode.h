#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <xArm6.h>
#include <RealVectorSpace.h>
#include <yaml-cpp/yaml.h>

class CameraCalibrationNode : public rclcpp::Node
{
public:
	CameraCalibrationNode(const std::string &config_file_path);

	void arucoSingleTFcallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
	void jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);

private:
  	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tf_subscription;
  	rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joints_state_subscription;
	Eigen::VectorXf joints_position;
    std::shared_ptr<robots::AbstractRobot> robot;
	KDL::Frame frame;
    Eigen::Vector3f aruco_bias;
};
