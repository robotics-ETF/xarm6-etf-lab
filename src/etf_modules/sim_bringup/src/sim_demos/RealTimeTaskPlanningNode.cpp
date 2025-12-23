#include "sim_demos/RealTimeTaskPlanningNode.h"

sim_bringup::RealTimeTaskPlanningNode::RealTimeTaskPlanningNode(const std::string &node_name, const std::string &config_file_path) : 
    TaskPlanningNode(node_name, config_file_path) 
{
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    dynamic_planner_config_file_path = node["planner"]["dynamic_planner_config_file_path"].as<std::string>();
    dynamic_node = YAML::LoadFile(project_abs_path + dynamic_planner_config_file_path);
    state = State::waiting;
}

void sim_bringup::RealTimeTaskPlanningNode::plannerSolving()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing dynamic planner...");
    // Update start and goal configuration
    file_out = std::ofstream(project_abs_path + dynamic_planner_config_file_path, std::ofstream::out);
    for (size_t i = 0; i < Robot::getNumDOFs(); i++)
    {
        dynamic_node["robot"]["q_start"][i] = Planner::scenario->getStart()->getCoord(i);
        dynamic_node["robot"]["q_goal"][i] = Planner::scenario->getGoal()->getCoord(i);
    }
    file_out << dynamic_node;
    file_out.close();

    dynamic_planner_thread = std::thread([this]() 
    {
        real_time_planning_node = std::make_shared<sim_bringup::RealTimePlanningNode>
                                    ("real_time_planning_node", dynamic_planner_config_file_path, false);
        rclcpp::spin(real_time_planning_node);
    });
    dynamic_planner_thread.detach();
    state = State::planning;
}

void sim_bringup::RealTimeTaskPlanningNode::plannerChecking()
{
    if (real_time_planning_node->getPlanningResult() == 1)
    {
        real_time_planning_node = nullptr;
        state = State::waiting;
        task = task_next;
    }
    else
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Dynamic planning is in progress...");
}

void sim_bringup::RealTimeTaskPlanningNode::executingTrajectory()
{
    // Trajectory is executed simultaneously during the process of planning.
    return;
}
