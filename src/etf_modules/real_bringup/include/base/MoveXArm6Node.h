#include "base/BaseNode.h"

namespace real_bringup
{
    class MoveXArm6Node : public real_bringup::BaseNode
    {
    public:
        MoveXArm6Node(const std::string node_name, const std::string config_file_path);

        std::shared_ptr<rclcpp::Node> set_position_node;
        rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr set_position_client;

    protected:
        void baseCallback() override { moveXArm6Callback(); }
        virtual void moveXArm6Callback();
        void goHome();
        void moveInJointSpace();
        void setPosition(const std::vector<float> &pose, float speed = 100, float acceleration = 1000);

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