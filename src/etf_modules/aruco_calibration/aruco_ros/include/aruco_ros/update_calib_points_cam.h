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

class TransformUpdateNode : public rclcpp::Node
{
public:
    TransformUpdateNode();
    void camera_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

private:
  	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_camera;
};