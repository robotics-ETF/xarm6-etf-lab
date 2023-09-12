#include "base/BaseNode.h"
#include "planning/Planner.h"
#include "environment/AABB.h"
#include "environment/Octomap.h"
#include "environment/ConvexHulls.h"

namespace sim_bringup
{
    class PlanningNode : public sim_bringup::BaseNode, 
                         public sim_bringup::Planner, 
                         public sim_bringup::AABB, 
                         public sim_bringup::Octomap, 
                         public sim_bringup::ConvexHulls
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