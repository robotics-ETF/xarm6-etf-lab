#include "demos/MoveXArm6Node.h"

#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <xarm_api/xarm_ros_client.h>
#include <xarm_msgs/srv/move_cartesian.hpp>
namespace real_bringup
{
    class MoveXArm6Node2 : public sim_bringup::MoveXArm6Node
    {
    public:
        MoveXArm6Node2(const std::string &node_name, const std::string &config_file_path);

        std::shared_ptr<rclcpp::Node> set_position_node;
        rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr set_position_client;

    protected:
        void moveXArm6Callback2() override { moveXArm6Callback(); }
        virtual void moveXArm6Callback2();
        void setPosition(const std::vector<float> &pose, float speed = 100, float acceleration = 1000);

        std::shared_ptr<rclcpp::Node> gripper_node;
        rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client;
        std::shared_ptr<rclcpp::Node> xarm_client_node;
        xarm_api::XArmROSClient xarm_client;

    private:
        enum State
        {
            going_home,
            moving_in_joint_space,
            setting_position1,
            setting_position2,
            closing_gripper,
            opening_gripper
        };
        State state;
    };
}