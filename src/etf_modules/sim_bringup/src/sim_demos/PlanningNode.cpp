#include "sim_demos/PlanningNode.h"

sim_bringup::PlanningNode::PlanningNode(const std::string &node_name, const std::string &config_file_path) : 
    BaseNode(node_name, config_file_path),
    AABB(config_file_path),
    Octomap(config_file_path),
    ConvexHulls(config_file_path)
{
    AABB::setEnvironment(Planner::scenario->getEnvironment());
    if (AABB::getMinNumCaptures() == 1)
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::callback, this, std::placeholders::_1));
    else
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::withFilteringCallback, this, std::placeholders::_1));

    // Octomap::marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>
    //     ("/occupied_cells_vis_array", 10);
    
    // ConvexHulls::points_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
    //     ("/convex_hulls", 10, std::bind(&ConvexHulls::pointsCallback, this, std::placeholders::_1));
    // ConvexHulls::polygons_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
    //     ("/convex_hulls_polygons", 10, std::bind(&ConvexHulls::polygonsCallback, this, std::placeholders::_1));
    
    state = waiting;
    q_start = Planner::scenario->getStart();
    q_goal = Planner::scenario->getGoal();
    path = {};
    planning_result = -1;
}

void sim_bringup::PlanningNode::planningCallback()
{
    switch (state)
    {
    case waiting:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting...");
        if (Robot::isReady() && AABB::isReady() && Planner::isReady())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating environment...");
            AABB::updateEnvironment();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning a path...");
            Planner::scenario->setStart(Robot::getJointsPositionPtr());
            std::thread planning_thread([this]() 
            {
                planning_result = Planner::solve();
            });
            planning_thread.detach();
            state = planning;
        }
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Waiting for the initial setup...");
        break;

    case planning:
        if (planning_result == 1)
        {
            Planner::preprocessPath(Planner::getPath(), path);
            Trajectory::clear();
            // Trajectory::addPath(path);
            Trajectory::addPath(path, false);
            Trajectory::publish();
            planning_result = -1;
            state = executing_trajectory;
        }
        else if (planning_result == 0)
        {
            planning_result = -1;
            state = waiting;
        }
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Waiting for the planner to finish...");
        break;

    case executing_trajectory:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing trajectory...");
        if (Robot::isReached(Planner::scenario->getGoal()))
        {
            rclcpp::shutdown();
            return;
        }
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------\n");
}
