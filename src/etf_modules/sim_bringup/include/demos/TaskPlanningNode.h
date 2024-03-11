//
// Created by nermin on 28.08.23.
//

#include "demos/PlanningNode.h"

namespace sim_bringup
{
    class TaskPlanningNode : public sim_bringup::PlanningNode
    {
    public:
        TaskPlanningNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void planningCallback() override { taskPlanningCallback(); }
        void taskPlanningCallback();
        bool computeObjectApproachAndPickStates();
        int chooseObject() override;
        bool whetherToRemove(const Eigen::Vector3f &object_pos, const Eigen::Vector3f &object_dim) override;

        enum Task 
        {
            waiting_for_object,
            choosing_object,
            computing_IK,
            going_towards_object,
            picking_object,
            raising_object,
            releasing_object,
            moving_object_to_destination,
            planning
        };
        Task task, task_next;

    private:
        std::shared_ptr<base::State> q_object_approach1, q_object_approach2;
        std::shared_ptr<base::State> q_object_pick;
        std::shared_ptr<base::State> q_goal;
        int obj_idx;
        int IK_computed;
        int picking_object_wait = 4;
        const float delta_z = 0.2, offset_z = 0;
        const Eigen::Vector3f goal_pos = Eigen::Vector3f(-0.5, 0, 0.2);
    };
}