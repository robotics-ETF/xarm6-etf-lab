#include "sim_demos/ReadTrajectory.h"


sim_bringup::ReadTrajectory::ReadTrajectory(const std::string &node_name, const std::string &config_file_path, bool loop_execution_) : 
    BaseNode(node_name, config_file_path),
    AABB(config_file_path)
{
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };

    if (AABB::getMinNumCaptures() == 1)
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::callback, this, std::placeholders::_1));
    else
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::withFilteringCallback, this, std::placeholders::_1));

    traj_file_node = YAML::LoadFile(project_abs_path + node["traj_file_path"].as<std::string>());
    loop_execution = loop_execution_;

    point_num = 0;
    YAML::Node point_node;
    while (true)
    {
        point_node = traj_file_node["point_" + std::to_string(point_num)];
        if (point_node.IsDefined())
        {
            Eigen::VectorXf q { Eigen::VectorXf::Zero(Robot::getNumDOFs()) };
            for (size_t idx = 0; idx < Robot::getNumDOFs(); idx++)
                q(idx) = point_node["q"][idx].as<float>();
            
            path.emplace_back(q);
            point_num += 2;
        }
        else
            break;
    }
    point_num = 0;
}

void sim_bringup::ReadTrajectory::planningCallback()
{
    if (!Robot::isReady())
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Waiting to set up the robot...");
        return;
    }
    if (!AABB::isReady())
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Waiting to set up the environment...");
        return;
    }

    if (point_num < path.size())
    {
        Trajectory::clear();
        Trajectory::addPoint(0, path[point_num]);
        Trajectory::publish(true);
        point_num++;
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal configuration is reached!");
    
    // std::cout << "----------------------------------------------------------------------------------------\n";
}
