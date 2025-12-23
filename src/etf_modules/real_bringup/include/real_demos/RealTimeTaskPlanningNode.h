//
// Created by nermin on 01.10.24.
//

#include "real_demos/TaskPlanningNode.h"
#include "real_demos/RealTimePlanningNode.h"

namespace real_bringup
{
    class RealTimeTaskPlanningNode : public real_bringup::TaskPlanningNode
    {
    public:
        RealTimeTaskPlanningNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void plannerSolving() override;
        void plannerChecking() override;
        void executingTrajectory() override;

        std::shared_ptr<sim_bringup::RealTimePlanningNode> real_time_planning_node;
        std::string dynamic_planner_config_file_path;
        YAML::Node dynamic_node;
        std::ofstream file_out;
        std::thread dynamic_planner_thread;
    };
}
