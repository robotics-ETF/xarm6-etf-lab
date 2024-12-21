#include "environments/AABB.h"

sim_bringup::AABB::AABB(const std::string &config_file_path)
{
    std::string project_abs_path(__FILE__);
    for (size_t i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    min_num_captures = node["cameras"]["min_num_captures"].as<size_t>();

    ready = false;
}

void sim_bringup::AABB::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    ready = false;
    resetMeasurements();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::moveFromROSMsg(*msg, *pcl);
    Eigen::Vector3f dim {};
    Eigen::Vector3f pos {};

    for (size_t i = 0; i < pcl->size(); i += 2)
    {
        dim << pcl->points[i].x, pcl->points[i].y, pcl->points[i].z;
        pos << pcl->points[i+1].x, pcl->points[i+1].y, pcl->points[i+1].z;
        dimensions.emplace_back(dim);
        positions.emplace_back(pos);
        num_captures.emplace_back(1);

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB %ld: dim = (%f, %f, %f), pos = (%f, %f, %f)",  // (x, y, z) in [m]
        //     i/2, dim.x(), dim.y(), dim.z(), pos.x(), pos.y(), pos.z());
    }
    ready = true;
}

void sim_bringup::AABB::withFilteringCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    ready = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
    pcl::moveFromROSMsg(*msg, *pcl);
    Eigen::Vector3f dim {};
    Eigen::Vector3f pos {};
    bool found { false };

    for (size_t i = 0; i < pcl->size(); i += 2)
    {
        dim << pcl->points[i].x, pcl->points[i].y, pcl->points[i].z;
        pos << pcl->points[i+1].x, pcl->points[i+1].y, pcl->points[i+1].z;

        if (whetherToRemove(pos, dim))
            continue;

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB %ld: dim = (%f, %f, %f), pos = (%f, %f, %f)",  // (x, y, z) in [m]
        //     i/2, dim.x(), dim.y(), dim.z(), pos.x(), pos.y(), pos.z());
    
        // Measurements are averaged online
        for (size_t j = 0; j < dimensions.size(); j++)
        {
            if ((dim - dimensions[j]).norm() < 0.05 && (pos - positions[j]).norm() < 0.05)
            {
                dimensions[j] = (num_captures[j] * dimensions[j] + dim) / (num_captures[j] + 1);
                positions[j] = (num_captures[j] * positions[j] + pos) / (num_captures[j] + 1);
                num_captures[j]++;
                found = true;
                break;
            }
        }
        
        if (!found)
        {
            dimensions.emplace_back(dim);
            positions.emplace_back(pos);
            num_captures.emplace_back(1);
        }                
    }
    ready = true;
}

bool sim_bringup::AABB::whetherToRemove([[maybe_unused]] const Eigen::Vector3f &object_pos, [[maybe_unused]] const Eigen::Vector3f &object_dim)
{    
    return false;
}

void sim_bringup::AABB::updateEnvironment(const std::shared_ptr<env::Environment> env, float max_obs_vel)
{
    if (!ready)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Environment cannot be updated!");
        return;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating environment...");
    env->removeObjects("ground", false);
    std::string label { max_obs_vel > 0 ? "dynamic_obstacle" : "static_obstacle" };
    
    for (size_t i = 0; i < positions.size(); i++)
    {
        if (num_captures[i] >= min_num_captures)
        {
		    std::shared_ptr<env::Object> object = 
                std::make_shared<env::Box>(dimensions[i], positions[i], fcl::Quaternionf::Identity(), label);
            object->setMaxVel(max_obs_vel);
            env->addObject(object);

            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB %ld: dim = (%f, %f, %f), pos = (%f, %f, %f), num. captures = %ld",
            //     i, dimensions[i].x(), dimensions[i].y(), dimensions[i].z(),                 // (x, y, z) in [m]
            //     positions[i].x(), positions[i].y(), positions[i].z(), num_captures[i]);     // (x, y, z) in [m]
        }
    }

    if (min_num_captures > 1)
        resetMeasurements();
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Environment is updated."); 
}

// Clear previous averaged measurements and start taking new ones
void sim_bringup::AABB::resetMeasurements()
{
    dimensions.clear();
    positions.clear();
    num_captures.clear();
}
