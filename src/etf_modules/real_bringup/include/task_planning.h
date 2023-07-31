#include "planning.h"

#include <xarm_api/xarm_ros_client.h>

class TaskPlanningNode : public PlanningNode
{
public:
    TaskPlanningNode(const std::string scenario_file_path, const std::string node_name, const int period, 
                     const std::string time_unit = "milliseconds");

protected:
    std::shared_ptr<rclcpp::Node> xarm_client_node;
    xarm_api::XArmROSClient xarm_client;

    void planningCallback() override { taskPlanningCallback(); }
    void taskPlanningCallback();

private:
    int task, task_next;
    Eigen::VectorXf object_angles_approach = Eigen::VectorXf(6);
};
