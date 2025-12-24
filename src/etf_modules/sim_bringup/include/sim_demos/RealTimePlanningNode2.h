//
// Created by nermin on 28.08.23.
//

#include "base/BaseNode.h"
#include "environments/AABB.h"

#include <rrtx/RRTx.h>

namespace sim_bringup
{
    class DynamicPlanningNode2 : public sim_bringup::BaseNode, 
                                 public sim_bringup::AABB, 
                                 public planning::rrtx::RRTx
    {
    public:
        DynamicPlanningNode2(const std::string &node_name, const std::string &config_file_path, bool loop_execution_);
        
        int getPlanningResult() const { return planning_result; }

    protected:
        void baseCallback() override { planningCallback(); }
        void planningCallback();
        virtual void computeTrajectory();

        bool iteration_completed;
        int planning_result;
        bool loop_execution;    // If true, after reaching the goal, start and goal will be switched, and algorithm will automatically continue its execution.
        std::shared_ptr<base::State> q_start_init;
        std::shared_ptr<base::State> q_goal_init;
        float trajectory_advance_time;
        float max_obs_vel;

        std::shared_ptr<base::State> q_rand;
        std::shared_ptr<base::State> q_near;
        std::shared_ptr<base::State> q_new;
        base::State::Status status;
        bool first_path_found;
    };
}
