#include "base/Planner.h"

sim_bringup::Planner::Planner(const std::string &config_file_path)
{
    std::string project_abs_path(__FILE__);
    for (size_t i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    try
    {
        YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
        YAML::Node planner_node { node["planner"] };
        if (!planner_node.IsDefined())
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Planner is not defined!");
            return;
        }
        
        ConfigurationReader::initConfiguration(project_abs_path + planner_node["configurations"].as<std::string>());
        planner = nullptr;

        std::string type { planner_node["type"].as<std::string>() };
        if (type == "RGBMT*")
            planner_type = planning::PlannerType::RGBMTStar;
        else if (type == "RGBT-Connect")
            planner_type = planning::PlannerType::RGBTConnect;
        else if (type == "RBT-Connect")
            planner_type = planning::PlannerType::RBTConnect;
        else if (type == "RRT-Connect")
            planner_type = planning::PlannerType::RRTConnect;

        YAML::Node max_planning_time_node { planner_node["max_planning_time"] };
        max_planning_time = (max_planning_time_node.IsDefined()) ? max_planning_time_node.as<float>() : INFINITY;

        if (planner_node["max_edge_length"].IsDefined())
            max_edge_length = planner_node["max_edge_length"].as<float>();
        else
        {
            max_edge_length = 0.1;
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Maximal edge length is not defined! Using default value of %f", max_edge_length);
        }
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << "\n";
    }
    
    ready = true;
}

/// @brief Solve a path planning problem from a start configuration 'q_start' to a goal configuration 'q_goal'
/// within a specified time limit 'max_planning_time_'.
/// @param q_start Start configuration
/// @param q_goal Goal configuration
/// @param max_planning_time_ Maximal planning time
/// @return Success of the planning, i.e., whether a feasible path from 'q_start' to 'q_goal' is found.
/// @note Not specified input arguments will use a corresponding values from a configuration yaml file.
bool sim_bringup::Planner::solve(std::shared_ptr<base::State> q_start, std::shared_ptr<base::State> q_goal, float max_planning_time_)
{
    ready = false;
    bool result { false };

    if (q_start == nullptr)
        q_start = scenario->getStart();
    else
        scenario->setStart(q_start);

    if (q_goal == nullptr)
        q_goal = scenario->getGoal();
    else
        scenario->setGoal(q_goal);

    if (max_planning_time_ != -1)
        max_planning_time = max_planning_time_;
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning the path..."); 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Number of collision objects: %ld", scenario->getEnvironment()->getNumObjects());
    switch (scenario->getNumDimensions())
    {
    case 6:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Start: (%f, %f, %f, %f, %f, %f)", q_start->getCoord(0), q_start->getCoord(1), 
            q_start->getCoord(2), q_start->getCoord(3), q_start->getCoord(4), q_start->getCoord(5));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Goal:  (%f, %f, %f, %f, %f, %f)", q_goal->getCoord(0), q_goal->getCoord(1), 
            q_goal->getCoord(2), q_goal->getCoord(3), q_goal->getCoord(4), q_goal->getCoord(5));
        break;
    
    default:
        break;
    }

    try
    {
        switch (planner_type)
        {
        case planning::PlannerType::RGBMTStar:
            RGBMTStarConfig::MAX_PLANNING_TIME = max_planning_time;
            planner = std::make_unique<planning::rbt_star::RGBMTStar>(scenario->getStateSpace(), q_start, q_goal);
            break;

        case planning::PlannerType::RGBTConnect:
            RGBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
            planner = std::make_unique<planning::rbt::RGBTConnect>(scenario->getStateSpace(), q_start, q_goal);
            break;
        
        case planning::PlannerType::RBTConnect:
            RBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
            planner = std::make_unique<planning::rbt::RBTConnect>(scenario->getStateSpace(), q_start, q_goal);
            break;

        case planning::PlannerType::RRTConnect:
            RRTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
            planner = std::make_unique<planning::rrt::RRTConnect>(scenario->getStateSpace(), q_start, q_goal);
            break;

        default:
            throw std::domain_error("The requested static planner is not found! ");
        }

        result = planner->solve();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Planning finished with %s ", 
            (result ? std::string("SUCCESS!").c_str() : std::string("FAILURE!").c_str()));
        if (result)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Number of states in the path: %ld", planner->getPath().size());
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Planning time: %f [ms]", planner->getPlannerInfo()->getPlanningTime() * 1e3);
            if (planner_type == planning::PlannerType::RGBMTStar)
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Path cost: %f", planner->getPlannerInfo()->getCostConvergence().back());
        }

        // Just for debugging (Not recommended to waste time!)
        // std::string project_abs_path(__FILE__);
        // for (size_t i = 0; i < 4; i++)
        //     project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
        // planner->outputPlannerData(project_abs_path + "/sim_bringup/data/planner_data.log");
    }
    catch (std::exception &e)
    {
        LOG(ERROR) << e.what();
    }

    ready = true;
    return result;
}

/// @brief Generate a new path 'new_path' from a path 'original_path' in a way that the distance between two adjacent nodes
/// is fixed (if possible) to a length of 'max_edge_length'. Geometrically, the new path remains the same as the original one,
/// but only their nodes may differ.
/// @param original_path Original path that will be transformed.
/// @param new_path New resulting path.
/// @param max_edge_length Maximal edge length (default value can be specified in a yaml file).
void sim_bringup::Planner::preprocessPath(const std::vector<std::shared_ptr<base::State>> &original_path, 
    std::vector<std::shared_ptr<base::State>> &new_path, float max_edge_length_)
{
    if (max_edge_length_ == -1)
        max_edge_length_ = max_edge_length;
    
    scenario->getStateSpace()->preprocessPath(original_path, new_path, max_edge_length_);
}
