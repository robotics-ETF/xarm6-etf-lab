#include "base.h"

#include <xarm_api/xarm_ros_client.h>
#include <xarm_msgs/srv/move_cartesian.hpp>

class MoveXArm6Node : public BaseNode
{
public:
    MoveXArm6Node(const std::string node_name, const int period, const std::string time_unit = "milliseconds");

protected:
    std::shared_ptr<rclcpp::Node> xarm_client_node;
    xarm_api::XArmROSClient xarm_client;
    rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr set_position_client;

    void baseCallback() override { moveXArm6Callback(); }
    void moveXArm6Callback();
    void goHome();
    void moveInJointSpace();
    void setPosition(const std::vector<float> &pose, float speed = 100, float acceleration = 100);
};
