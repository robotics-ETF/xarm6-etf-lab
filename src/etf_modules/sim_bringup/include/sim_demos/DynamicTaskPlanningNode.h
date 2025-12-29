//
// Created by nermin on 12.05.24.
//

#include "sim_demos/TaskPlanningNode.h"
#include "sim_demos/DynamicPlanningNode.h"

namespace sim_bringup
{
    class DynamicTaskPlanningNode : public sim_bringup::TaskPlanningNode
    {
    public:
        DynamicTaskPlanningNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void plannerSolving() override;
        void plannerChecking() override;
        void executingTrajectory() override;

        std::shared_ptr<sim_bringup::DynamicPlanningNode> dynamic_planning_node;
        std::string dynamic_planner_config_file_path;
        YAML::Node dynamic_node;
        std::ofstream file_out;
        std::thread dynamic_planner_thread;
    };
}
