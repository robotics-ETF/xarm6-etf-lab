#include "planning.h"

#include <xarm_api/xarm_ros_client.h>

class TaskPlanningNode : public PlanningNode
{
public:
    TaskPlanningNode(const std::string scenario_file_path, const std::string node_name, const int period, 
                     const std::string time_unit = "milliseconds");

protected:
    void planningCallback() override { taskPlanningCallback(); }
    void taskPlanningCallback();
    void boundingBoxesCallbackWithFiltering(const sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
    void chooseObject();
    bool computeObjectApproachAndPickAngles();
    bool whetherToRemoveBoundingBox(Eigen::Vector3f &object_pos, Eigen::Vector3f &object_dim) override;

private:
    std::shared_ptr<rclcpp::Node> xarm_client_node;
    xarm_api::XArmROSClient xarm_client;
    int task, task_next;
    std::shared_ptr<base::State> q_object_approach1, q_object_approach2;
    std::shared_ptr<base::State> q_object_pick;
    std::shared_ptr<base::State> q_goal;
    int obj_idx;
    int IK_computed;
    const float delta_z = 0.2, offset_z = 0;
    const Eigen::Vector3f goal_pos = Eigen::Vector3f(-0.5, 0, 0.2);
};
