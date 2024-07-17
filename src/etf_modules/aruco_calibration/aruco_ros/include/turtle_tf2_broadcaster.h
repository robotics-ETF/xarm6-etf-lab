#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <xArm6.h>
#include <RealVectorSpace.h>
#include <yaml-cpp/yaml.h>

#include <sstream>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2_kdl/tf2_kdl.h"

class FramePublisher : public rclcpp::Node
{
public:
	FramePublisher();
    void jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
	void handle_turtle_pose(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

private:
  	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tf_subscription;
  	rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joints_state_subscription;
	Eigen::VectorXf joints_position;
    std::shared_ptr<robots::AbstractRobot> robot;
	KDL::Frame frame;
    Eigen::Vector3f aruco_bias;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
  	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
