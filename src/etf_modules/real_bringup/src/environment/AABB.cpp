#include "environment/AABB.h"

real_bringup::AABB::AABB(const std::string config_file_path)
{
    std::string project_abs_path = std::string(__FILE__);
    for (int i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node = YAML::LoadFile(project_abs_path + config_file_path);
    min_num_captures = node["cameras"]["min_num_captures"].as<int>();
}

void real_bringup::AABB::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    resetMeasurements();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::moveFromROSMsg(*msg, *pcl);
    for (int i = 0; i < pcl->size(); i += 2)
    {
        pcl::PointXYZ dim = pcl->points[i];
        pcl::PointXYZ pos = pcl->points[i+1];
        dimensions.emplace_back(fcl::Vector3f(dim.x, dim.y, dim.z));
        positions.emplace_back(fcl::Vector3f(pos.x, pos.y, pos.z));
        num_captures.emplace_back(1);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB %d: dim = (%f, %f, %f), pos = (%f, %f, %f)",
        //     i/2, dim.x, dim.y, dim.z, pos.x, pos.y, pos.z);
    }
}

void real_bringup::AABB::withFilteringCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
    pcl::moveFromROSMsg(*msg, *pcl);
    Eigen::Vector3f dim;
    Eigen::Vector3f pos;
    bool found = false;

    for (int i = 0; i < pcl->size(); i += 2)
    {
        dim << pcl->points[i].x, pcl->points[i].y, pcl->points[i].z;
        pos << pcl->points[i+1].x, pcl->points[i+1].y, pcl->points[i+1].z;

        if (whetherToRemove(pos, dim))
            continue;

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB %d: dim = (%f, %f, %f), pos = (%f, %f, %f)",  // (x, y, z) in [m]
        //     i/2, dim.x(), dim.y(), dim.z(), pos.x(), pos.y(), pos.z());
    
        // Measurements are averaged online
        for (int j = 0; j < dimensions.size(); j++)
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
}

bool real_bringup::AABB::whetherToRemove(Eigen::Vector3f &object_pos, Eigen::Vector3f &object_dim)
{    
    return false;
}

void real_bringup::AABB::updateEnvironment()
{
    env->removeCollisionObjects(1);  // table (idx = 0) is preserved, and all other objects are deleted
    
    Eigen::Matrix3f rot = fcl::Quaternionf(0, 0, 0, 0).matrix();
    for (int i = 0; i < positions.size(); i++)
    {
        if (num_captures[i] >= min_num_captures)
        {
            std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(dimensions[i]);
            std::shared_ptr<fcl::CollisionObjectf> ob = std::make_shared<fcl::CollisionObjectf>(box, rot, positions[i]);
            env->addCollisionObject(ob);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB %d: dim = (%f, %f, %f), pos = (%f, %f, %f), num. captures = %d",
                i, dimensions[i].x(), dimensions[i].y(), dimensions[i].z(),                 // (x, y, z) in [m]
                positions[i].x(), positions[i].y(), positions[i].z(), num_captures[i]);     // (x, y, z) in [m]
        }
    }

    // Predefined fixed obstacle
    // std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(fcl::Vector3f(0.1, 0.1, 0.3));
    // std::shared_ptr<fcl::CollisionObjectf> ob = std::make_shared<fcl::CollisionObjectf>(box, rot, fcl::Vector3f(0.2, 0.2, 0.15));
    // env->addCollisionObject(ob);

    resetMeasurements();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Environment is updated."); 
}

// Clear previous averaged measurements and start taking new ones
void real_bringup::AABB::resetMeasurements()
{
    dimensions.clear();
    positions.clear();
    num_captures.clear();
}
