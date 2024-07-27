//
// Created by nermin on 28.08.23.
//

#include "base/BaseNode.h"
#include "environments/AABB.h"

#include <DRGBT.h>
#include <thread>

namespace sim_bringup
{
    class RealTimePlanningNode : public sim_bringup::BaseNode, 
                                 public sim_bringup::AABB, 
                                 public planning::drbt::DRGBT
    {
    public:
        RealTimePlanningNode(const std::string &node_name, const std::string &config_file_path, bool loop_execution_,
                             const std::string &output_file_name = "");

        int getPlanningResult() const { return planning_result; }
        
    protected:
        void baseCallback() override { planningCallback(); }
        void planningCallback();
        void taskComputingNextConfiguration();
        void taskReplanning();
        void replan(float max_planning_time) override;
        virtual void computeTrajectory();
        void recordingTrajectoryCallback();

        int planning_result;
        int replanning_result;  //  0: replanning was not successful
                                //  1: replanning was successful and predefined path needs to be updated
                                // -1: replanning was successful but predefined path does not need to be updated
        bool loop_execution;    // If true, after reaching the goal, start and goal will be switched, and algorithm will automatically continue its execution.
        std::shared_ptr<base::State> q_start_init;
        std::shared_ptr<base::State> q_goal_init;
        rclcpp::TimerBase::SharedPtr recording_trajectory_timer;
        std::ofstream output_file;
        Eigen::VectorXf max_error;
    };
}
