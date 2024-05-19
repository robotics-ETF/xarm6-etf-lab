#include "sim_demos/RealTimeTaskPlanningNode.h"

sim_bringup::RealTimeTaskPlanningNode::RealTimeTaskPlanningNode(const std::string &node_name, const std::string &config_file_path) : 
    TaskPlanningNode(node_name, config_file_path) 
{
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    dynamic_planner_config_file_path = node["planner"]["dynamic_planner_config_file_path"].as<std::string>();
    dynamic_node = YAML::LoadFile(project_abs_path + dynamic_planner_config_file_path);
}

void sim_bringup::RealTimeTaskPlanningNode::planningCase()
{
    switch (state)
    {
    case State::waiting:
        break;
    
    case State::planning:
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
            real_time_planning_node = std::make_shared<sim_bringup::RealTimePlanningNode>("real_time_planning_node", dynamic_planner_config_file_path);
            executor.add_node(real_time_planning_node);
            executor.spin();
        });
        dynamic_planner_thread.detach();
        state = State::executing_trajectory;        
        break;
    
    case State::executing_trajectory:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing trajectory...");
        if (Robot::isReached(Planner::scenario->getGoal()))
        {
            executor.cancel();
            executor.remove_node(real_time_planning_node);
            real_time_planning_node = nullptr;
            state = State::planning;
            task = task_next;
        }
        break;
    }
}
