//
// Created by nermin on 12.05.24.
//

#include "sim_demos/TaskPlanningNode.h"
#include "sim_demos/RealTimePlanningNode.h"

namespace sim_bringup
{
    class RealTimeTaskPlanningNode : public sim_bringup::TaskPlanningNode
    {
    public:
        RealTimeTaskPlanningNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void planningCase() override;

        std::shared_ptr<sim_bringup::RealTimePlanningNode> real_time_planning_node;

    private:
        std::string dynamic_planner_config_file_path;
        YAML::Node dynamic_node;
        std::ofstream file_out;
        rclcpp::executors::MultiThreadedExecutor executor;
        std::thread dynamic_planner_thread;
    };
}
