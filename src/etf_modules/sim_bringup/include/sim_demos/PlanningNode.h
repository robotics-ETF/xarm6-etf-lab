//
// Created by nermin on 28.08.23.
//

#include "base/BaseNode.h"
#include "environments/AABB.h"
#include "environments/Octomap.h"
#include "environments/ConvexHulls.h"

#include <thread>

namespace sim_bringup
{
    class PlanningNode : public sim_bringup::BaseNode,
                         public sim_bringup::AABB,
                         public sim_bringup::Octomap,
                         public sim_bringup::ConvexHulls
    {
    public:
        PlanningNode(const std::string &node_name, const std::string &config_file_path, bool loop_execution_);

    protected:
        virtual void baseCallback() override { planningCallback(); }
        virtual void planningCallback();

        enum State
        {
            waiting,
            planning,
            executing_trajectory
        };
        State state;

        std::shared_ptr<base::State> q_start;
        std::shared_ptr<base::State> q_goal;
        std::vector<std::shared_ptr<base::State>> path;
        int planning_result;
        bool loop_execution;    // If true, after reaching the goal, start and goal will be switched, and algorithm will automatically continue its execution.
    };
}
