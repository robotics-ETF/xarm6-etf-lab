//
// Created by nermin on 12.07.25.
//

#include "base/BaseNode.h"
#include "environments/AABB.h"

namespace sim_bringup
{
    class ReadTrajectory : public sim_bringup::BaseNode,
                           public sim_bringup::AABB
    {
    public:
        ReadTrajectory(const std::string &node_name, const std::string &config_file_path, bool loop_execution_);

    protected:
        void baseCallback() override { planningCallback(); }
        void planningCallback();

        bool loop_execution;    // If true, after reaching the goal, start and goal will be switched, and algorithm will automatically continue its execution.
        YAML::Node traj_file_node;
        size_t point_num;
        std::vector<Eigen::VectorXf> path;
    };
}
