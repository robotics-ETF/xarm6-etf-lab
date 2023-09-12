#include "base/BaseNode.h"
#include "planning/Planner.h"
#include "environment/AABB.h"

namespace real_bringup
{
    class PlanningNode : public real_bringup::BaseNode, 
                         public real_bringup::Planner, 
                         public real_bringup::AABB
    {
    public:
        PlanningNode(const std::string node_name, const std::string config_file_path);

    protected:
        virtual void baseCallback() override { planningCallback(); }
        virtual void planningCallback();

        std::shared_ptr<scenario::Scenario> scenario;

        enum State
        {
            waiting,
            planning,
            publishing_trajectory,
            executing_trajectory
        };
        State state;
    };
}