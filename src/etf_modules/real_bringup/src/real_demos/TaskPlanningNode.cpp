#include "real_demos/TaskPlanningNode.h"

real_bringup::TaskPlanningNode::TaskPlanningNode(const std::string &node_name, const std::string &config_file_path) :
    sim_bringup::TaskPlanningNode(node_name, config_file_path) 
{
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    YAML::Node scenario_node { node["scenario"] };

    picking_object_wait_max = scenario_node["picking_object_wait_max"].as<size_t>();
    picking_object_wait = picking_object_wait_max;
    opened_gripper_pos = scenario_node["opened_gripper_pos"].as<float>();
    closed_gripper_pos = scenario_node["closed_gripper_pos"].as<float>();
    gripper_pos = opened_gripper_pos;

    xarm_client_node = std::make_shared<rclcpp::Node>("xarm_client_node");
    xarm_client.init(xarm_client_node, "xarm");
    xarm_client.set_gripper_enable(true);
    xarm_client.set_gripper_mode(0);
    xarm_client.set_gripper_speed(scenario_node["max_gripper_speed"].as<float>());
    xarm_client.set_gripper_position(gripper_pos);  // The gripper is open by default
}

void real_bringup::TaskPlanningNode::taskPlanningCallback()
{
    switch (task)
    {
    case waiting_for_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the object...");
        AABB::resetMeasurements();
        if (Robot::isReady() && AABB::isReady())
            task = choosing_object;
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Robot or environment is not ready...");
        break;

    case choosing_object:
        obj_idx = chooseObject();
        if (obj_idx != -1)
        {
            task = computing_IK;
            IK_computed = computeObjectApproachAndPickStates();
        }
        else
            task = waiting_for_object;
        break;

    case computing_IK:
        if (IK_computed == 1)
        {
            AABB::resetMeasurements();
            scenario->setStart(Robot::getJointsPositionPtr());
            scenario->setGoal(q_object_approach1);
            task = planning;
            task_next = going_towards_object;
            IK_computed = -1;
        }
        else if (IK_computed == 0)
            task = waiting_for_object;
        
        break;

    case going_towards_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going towards the object...");
        xarm_client.set_gripper_position(opened_gripper_pos);
        Trajectory::clear();
        Trajectory::addTrajectory(Planner::convertPathToTraj({q_object_approach1, q_object_approach2, q_object_pick}));
        Trajectory::publish();
        task = picking_object;
        break;

    case picking_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Picking the object...");
        picking_object_wait--;
        if (picking_object_wait == 1)
            xarm_client.set_gripper_position(closed_gripper_pos);
        else if (picking_object_wait == 0)
        {
            picking_object_wait = picking_object_wait_max;
            task = raising_object;
        }
        break;

    case raising_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the object...");
        Trajectory::clear();
        Trajectory::addTrajectory(Planner::convertPathToTraj({q_object_pick, q_object_approach1}));
        Trajectory::publish();
        task = moving_object_to_destination;
        break;

    case moving_object_to_destination:
        xarm_client.get_gripper_position(&gripper_pos);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper position: %f", gripper_pos);
        if (gripper_pos <= closed_gripper_pos + 10)   // Nothing is caught
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to pick the object again...");
            xarm_client.set_gripper_position(opened_gripper_pos);
            task = waiting_for_object;     // Go from the beginning
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the object to destination...");
            AABB::resetMeasurements();
            q_object_approach1 = Planner::scenario->getStateSpace()->getNewState(q_object_approach1->getCoord());   // Reset all additional data set before for 'q_object_approach1'
            scenario->setStart(q_object_approach1);
            scenario->setGoal(q_goal);
            task = planning;
            task_next = releasing_object;
        }
        break;
    
    case releasing_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the object...");
        xarm_client.set_gripper_position(opened_gripper_pos);
        task = waiting_for_object;
        break;

    case planning:
        planningCase();
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------\n");
}
