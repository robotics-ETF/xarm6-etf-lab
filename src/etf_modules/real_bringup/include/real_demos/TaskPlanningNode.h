#include "sim_demos/TaskPlanningNode.h"

#include <xarm_api/xarm_ros_client.h>

namespace real_bringup
{
    class TaskPlanningNode : public sim_bringup::TaskPlanningNode
    {
    public:
        TaskPlanningNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void taskPlanningCallback() override;

        std::shared_ptr<rclcpp::Node> xarm_client_node;
        xarm_api::XArmROSClient xarm_client;

        size_t picking_object_wait;
        size_t picking_object_wait_max;
        float gripper_pos;
        float opened_gripper_pos;
        float closed_gripper_pos;
    };
}
