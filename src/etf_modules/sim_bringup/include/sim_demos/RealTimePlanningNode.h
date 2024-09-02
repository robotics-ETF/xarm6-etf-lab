//
// Created by nermin on 28.08.23.
//

#include "base/BaseNode.h"
#include "environments/AABB.h"

#include <DRGBT.h>
#include <std_srvs/srv/empty.hpp>

namespace sim_bringup
{
    class RealTimePlanningNode : public sim_bringup::BaseNode, 
                                 public sim_bringup::AABB, 
                                 public planning::drbt::DRGBT
    {
    public:
        RealTimePlanningNode(const std::string &node_name, const std::string &config_file_path, bool loop_execution_,
                             const std::string &output_file_name = "");
        
    protected:
        void baseCallback() override { planningCallback(); }
        void planningCallback();
        void taskComputingNextConfiguration();
        void taskReplanning(bool replanning_required_explicitly = false);
        void replanningCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                const std::shared_ptr<std_srvs::srv::Empty::Response> response);
        virtual void computeTrajectory();

        bool iteration_completed;
        int planning_result;
        int replanning_result;  //  0: replanning was not successful
                                //  1: replanning was successful and predefined path needs to be updated
                                // -1: replanning was successful but predefined path does not need to be updated
        bool loop_execution;    // If true, after reaching the goal, start and goal will be switched, and algorithm will automatically continue its execution.
        std::shared_ptr<base::State> q_start_init;
        std::shared_ptr<base::State> q_goal_init;
        float trajectory_advance_time;

        rclcpp::CallbackGroup::SharedPtr callback_group;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr replanning_service;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr replanning_client;
    
    private:
        void recordingTrajectoryCallback();

        rclcpp::TimerBase::SharedPtr recording_trajectory_timer;
        std::ofstream output_file;
        Eigen::VectorXf max_error;
    };
}
