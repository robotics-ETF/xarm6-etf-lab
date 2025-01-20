#include "environments/Obstacles.h"

perception_etflab::Obstacles::Obstacles(const std::string &config_file_path)
{
    if (config_file_path.find("sim") == std::string::npos)
		return;

    // All these settings are only for simulated robot in order to be able to deal with simulated obstacles
    std::string project_abs_path(__FILE__);
	for (size_t i = 0; i < 4; i++)
		project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
    
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };

    YAML::Node obs_node { node["random_obstacles"] };
    num_obstacles = obs_node["num"].as<size_t>();
	max_vel = obs_node["max_vel"].as<float>();
	for (size_t i = 0; i < 3; i++)
		dim(i) = obs_node["dim"][i].as<float>();
    period = obs_node["period"].as<float>();

    YAML::Node robot_node { node["robot"] };
    ground_included = robot_node["ground_included"].as<size_t>();
    for (size_t i = 0; i < 3; i++)
        WS_center(i) = robot_node["WS_center"][i].as<float>();

    WS_radius = robot_node["WS_radius"].as<float>();
    base_radius = std::max(robot_node["capsules_radius"][0].as<float>(), robot_node["capsules_radius"][1].as<float>()) + dim.norm();
    robot_max_vel = robot_node["max_vel_first_joint"].as<float>();

    // Initial settings for each obstacle
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Adding %ld random obstacles...", num_obstacles);
    Eigen::Vector3f pos {}, vel {};
    float r {}, fi {}, theta {};

    for (size_t i = 0; i < num_obstacles; i++)
    {
        r = float(rand()) / RAND_MAX * WS_radius;
        fi = float(rand()) / RAND_MAX * 2 * M_PI;
        theta = float(rand()) / RAND_MAX * M_PI;
		pos.x() = WS_center.x() + r * std::cos(fi) * std::sin(theta);
		pos.y() = WS_center.y() + r * std::sin(fi) * std::sin(theta);
		pos.z() = WS_center.z() + r * std::cos(theta);

        vel = Eigen::Vector3f::Random(3);
        vel.normalize();
		vel *= float(rand()) / RAND_MAX * max_vel;

        if (!isValid(pos, vel.norm()))   
            i--;
        else
        {
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            cluster->emplace_back(pcl::PointXYZRGB(pos.x(), pos.y(), pos.z()));
            cluster->emplace_back(pcl::PointXYZRGB(pos.x() - dim.x()/2, pos.y() - dim.y()/2, pos.z() - dim.z()/2));
            cluster->emplace_back(pcl::PointXYZRGB(pos.x() + dim.x()/2, pos.y() + dim.y()/2, pos.z() + dim.z()/2));
            obstacles.emplace_back(cluster);
            velocities.emplace_back(vel);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%ld. Obstacle pos: (%f, %f, %f)", i, pos.x(), pos.y(), pos.z());
        }
    }
}

// Reflect obstacles in random directions
// void perception_etflab::Obstacles::move(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
// {
//     std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster { nullptr };
//     Eigen::Vector3f pos {}, vel {};
//     float vel_intensity {};

//     for (size_t i = 0; i < num_obstacles; i++)
//     {
//         cluster = obstacles[i];
//         pos = Eigen::Vector3f(cluster->points.front().x, cluster->points.front().y, cluster->points.front().z);
//         pos += velocities[i] * period;
//         vel_intensity = velocities[i].norm();

//         if (!isValid(pos, vel_intensity))
//         {
//             // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid object position. Computing new velocity. ");
//             vel = Eigen::Vector3f::Random(3);
//             vel.normalize();
//             velocities[i] = vel_intensity * vel;
//             i--;
//         }
//         else
//         {
//             for (pcl::PointCloud<pcl::PointXYZRGB>::iterator point = cluster->begin(); point < cluster->end(); point++)
//             {
//                 point->x += velocities[i].x() * period;
//                 point->y += velocities[i].y() * period;
//                 point->z += velocities[i].z() * period;
//             }
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%ld. Obstacle pos: (%f, %f, %f)", i, pos.x(), pos.y(), pos.z());
//         }
//     }
//     clusters = obstacles;
// }

// Reflect obstacles according to the principle of light reflecting
void perception_etflab::Obstacles::move(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster { nullptr };
    float tol_radius {}, t_param {};
    Eigen::Vector3f pos {}, pos_next {}, vec_normal {};
    bool change { true };

    for (size_t i = 0; i < num_obstacles; i++)
    {
        cluster = obstacles[i];
        tol_radius = std::max(velocities[i].norm() / robot_max_vel, base_radius);
        pos = Eigen::Vector3f(cluster->points.front().x, cluster->points.front().y, cluster->points.front().z);
        pos_next = pos + velocities[i] * period;
        change = true;
        
        if (pos_next.z() < 0)
            vec_normal << 0, 0, 1;
        else if ((pos_next - WS_center).norm() > WS_radius)
            vec_normal << -pos_next.x(), -pos_next.y(), -(pos_next.z() - WS_center.z());
        else if (pos_next.head(2).norm() < tol_radius && pos_next.z() < WS_center.z())
            vec_normal << pos_next.x(), pos_next.y(), 0;
        else if ((pos_next - WS_center).norm() < tol_radius)
            vec_normal << pos_next.x(), pos_next.y(), pos_next.z() - WS_center.z();
        else
        {
            pos = pos_next;
            change = false;
        }

        if (change)
        {
            t_param = (pos_next - pos).dot(vec_normal) / vec_normal.squaredNorm();
            pos = 2*pos_next - pos - 2*t_param * vec_normal;
            velocities[i] = (pos - pos_next) / period;
        }
        
        for (pcl::PointCloud<pcl::PointXYZRGB>::iterator point = cluster->begin(); point < cluster->end(); point++)
        {
            point->x += velocities[i].x() * period;
            point->y += velocities[i].y() * period;
            point->z += velocities[i].z() * period;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%ld. Obstacle pos: (%f, %f, %f)", i, pos.x(), pos.y(), pos.z());
    }
    clusters = obstacles;
}

void perception_etflab::Obstacles::move([[maybe_unused]] pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl)
{
}

// Check whether an object position 'pos' is valid when the object moves at 'vel' velocity
bool perception_etflab::Obstacles::isValid(const Eigen::Vector3f &pos, float vel)
{
    float tol_radius { std::max(vel / robot_max_vel, base_radius) };

    if (ground_included > 0)
    {
        if ((pos - WS_center).norm() > WS_radius || pos.z() < 0 ||              // Out of workspace
            (pos.head(2).norm() < tol_radius && pos.z() < WS_center.z()) ||     // Surrounding of robot base
            (pos - WS_center).norm() < tol_radius)                              // Surrounding of robot base
            return false;
    }
    else
    {
        if ((pos - WS_center).norm() > WS_radius ||                                                     // Out of workspace
            (pos.head(2).norm() < tol_radius && pos.z() < WS_center.z() && pos.z() > -base_radius) ||   // Surrounding of robot base
            (pos - WS_center).norm() < tol_radius)                                                      // Surrounding of robot base
            return false;
    }

    return true;
}
