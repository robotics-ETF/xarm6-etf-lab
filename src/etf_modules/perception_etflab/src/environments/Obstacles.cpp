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
    YAML::Node node2 { YAML::LoadFile(project_abs_path + config_file_path.substr(0, config_file_path.find_last_of("/\\")) + "/random_scenarios.yaml") };
    Eigen::Vector3f pos {}, vel {}, dim {};
    num_obstacles = node["environment"].size();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Adding %ld predefined obstacles...", num_obstacles);

    for (size_t i = 0; i < num_obstacles; i++)
    {
        YAML::Node obstacle = node["environment"][i];
        YAML::Node d = obstacle["box"]["dim"];
        YAML::Node p = obstacle["box"]["pos"];
        YAML::Node r = obstacle["box"]["rot"];
        YAML::Node min_dist_tol = obstacle["box"]["min_dist_tol"];
        
        dim << d[0].as<float>(), d[1].as<float>(), d[2].as<float>();
        pos << p[0].as<float>(), p[1].as<float>(), p[2].as<float>();

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        cluster->emplace_back(pcl::PointXYZRGB(pos.x(), pos.y(), pos.z()));
        cluster->emplace_back(pcl::PointXYZRGB(pos.x() - dim.x()/2, pos.y() - dim.y()/2, pos.z() - dim.z()/2));
        cluster->emplace_back(pcl::PointXYZRGB(pos.x() + dim.x()/2, pos.y() + dim.y()/2, pos.z() + dim.z()/2));
        obstacles.emplace_back(cluster);
        velocities.emplace_back(vel);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%ld. Obstacle pos: (%f, %f, %f)", i, pos.x(), pos.y(), pos.z());
    }
    
    sign = Eigen::VectorXi::Ones(num_obstacles);
    path_len = Eigen::VectorXf::Zero(num_obstacles);
    max_vel = node["perception"]["max_vel"].as<float>();
    period = node["perception"]["period"].as<float>();

    num_rand_obstacles = node["random_obstacles"]["num"].as<size_t>();
    if (num_rand_obstacles > 0)
    {
        for (size_t i = 0; i < 3; i++)
            dim_rand(i) = node["random_obstacles"]["dim"][i].as<float>();

        YAML::Node robot_node { node["robot"] };
        ground_included = robot_node["ground_included"].as<size_t>();
        for (size_t i = 0; i < 3; i++)
            WS_center(i) = robot_node["WS_center"][i].as<float>();

        WS_radius = robot_node["WS_radius"].as<float>();
        base_radius = std::max(robot_node["capsules_radius"][0].as<float>(), robot_node["capsules_radius"][1].as<float>()) + dim_rand.norm();
        robot_max_vel = robot_node["max_vel_first_joint"].as<float>();
        int num_run = node["random_obstacles"]["num_run"].as<int>();

        if (num_run == -1)      // First option (generating here in the code):
        {
            float r {}, fi {}, theta {};
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Adding %ld random obstacles...", num_rand_obstacles);
            for (size_t i = 0; i < num_rand_obstacles; i++)
            {
                // Initial settings for each random obstacle
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
                    cluster->emplace_back(pcl::PointXYZRGB(pos.x() - dim_rand.x()/2, pos.y() - dim_rand.y()/2, pos.z() - dim_rand.z()/2));
                    cluster->emplace_back(pcl::PointXYZRGB(pos.x() + dim_rand.x()/2, pos.y() + dim_rand.y()/2, pos.z() + dim_rand.z()/2));
                    obstacles.emplace_back(cluster);
                    velocities.emplace_back(vel);

                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%ld. Obstacle pos: (%f, %f, %f)", i, pos.x(), pos.y(), pos.z());
                }
            }
        }
        else    // Second option (reading from a yaml file):
        {
            for (size_t j = 0; j < num_rand_obstacles; j++)
            {
                for (size_t i = 0; i < 3; i++)
                {
                    pos(i) = node2["scenario_" + std::to_string(num_rand_obstacles)]["run_" + std::to_string(num_run-1)]
                                ["object_" + std::to_string(j)]["pos"][i].as<float>();
                    vel(i) = node2["scenario_" + std::to_string(num_rand_obstacles)]["run_" + std::to_string(num_run-1)]
                                ["object_" + std::to_string(j)]["vel"][i].as<float>();
                }
                
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                cluster->emplace_back(pcl::PointXYZRGB(pos.x(), pos.y(), pos.z()));
                cluster->emplace_back(pcl::PointXYZRGB(pos.x() - dim_rand.x()/2, pos.y() - dim_rand.y()/2, pos.z() - dim_rand.z()/2));
                cluster->emplace_back(pcl::PointXYZRGB(pos.x() + dim_rand.x()/2, pos.y() + dim_rand.y()/2, pos.z() + dim_rand.z()/2));
                obstacles.emplace_back(cluster);
                velocities.emplace_back(vel*0.5);

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%ld. Obstacle pos: (%f, %f, %f)", j, pos.x(), pos.y(), pos.z());
            }
        }
    }
    // ------------------------------------------------------------------------------- //
	
    if (node["perception"]["motion_type"].as<std::string>() == "circular")
        motion_type = MotionType::circular;
    else if (node["perception"]["motion_type"].as<std::string>() == "two_tunnels")
        motion_type = MotionType::two_tunnels;
    else if (node["perception"]["motion_type"].as<std::string>() == "random_directions")
        motion_type = MotionType::random_directions;
    else if (node["perception"]["motion_type"].as<std::string>() == "light_directions")
        motion_type = MotionType::light_directions;
    else
    {
        motion_type = MotionType::light_directions;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motion type is not specified! Using 'light_directions'.");
    }
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

// ------------------------------------- Add the law for obstacles motion here ------------------------------------- //

void perception_etflab::Obstacles::move(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    switch (motion_type)
    {
    case MotionType::circular:
        moveCircular(clusters);
        break;
    
    case MotionType::two_tunnels:
        moveTwoTunnels(clusters);
        break;

    case MotionType::random_directions:
        moveRandomDirections(clusters);
        break;

    case MotionType::light_directions:
        moveLightDirections(clusters);
        break;

    default:
        break;
    }
}

void perception_etflab::Obstacles::moveCircular(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster { nullptr };
    Eigen::Vector3f pos {}, pos_prev {};
    float radius {}, phi {}, delta_phi {};

    for (size_t i = 0; i < num_obstacles; i++)
    {
        cluster = obstacles[i];
        pos_prev = Eigen::Vector3f(cluster->points.front().x, cluster->points.front().y, cluster->points.front().z);
        radius = pos_prev.head(2).norm();
        delta_phi = max_vel / radius * period;
        phi = std::atan2(pos_prev.y(), pos_prev.x());
        pos.x() = radius * std::cos(phi + delta_phi);
        pos.y() = radius * std::sin(phi + delta_phi);
        pos.z() = pos_prev.z();
        velocities[i] = (pos - pos_prev) / period;
        
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

void perception_etflab::Obstacles::moveTwoTunnels(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster { nullptr };
    Eigen::Vector3f pos {};
    const float path_len_max { 0.2 };

    for (size_t i = 0; i < num_obstacles; i++)
    {
        if (path_len(i) > path_len_max)
        {
            sign(i) *= -1;
            path_len(i) = -path_len_max;
        }
        
        cluster = obstacles[i];
        pos = Eigen::Vector3f(cluster->points.front().x, cluster->points.front().y, cluster->points.front().z);
        if (pos.y() > 0)
            velocities[i] = Eigen::Vector3f::UnitX() * sign(i) * max_vel;     // Move along x-axis
        else
            velocities[i] = Eigen::Vector3f::UnitY() * sign(i) * max_vel;     // Move along y-axis
        
        path_len(i) += velocities[i].norm() * period;
        
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

// Reflect obstacles in random directions
void perception_etflab::Obstacles::moveRandomDirections(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster { nullptr };
    Eigen::Vector3f pos {}, vel {};
    float vel_intensity {};

    for (size_t i = 0; i < num_rand_obstacles; i++)
    {
        cluster = obstacles[i];
        pos = Eigen::Vector3f(cluster->points.front().x, cluster->points.front().y, cluster->points.front().z);
        pos += velocities[i] * period;
        vel_intensity = velocities[i].norm();

        if (!isValid(pos, vel_intensity))
        {
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid object position. Computing new velocity. ");
            vel = Eigen::Vector3f::Random(3);
            vel.normalize();
            velocities[i] = vel_intensity * vel;
            i--;
        }
        else
        {
            for (pcl::PointCloud<pcl::PointXYZRGB>::iterator point = cluster->begin(); point < cluster->end(); point++)
            {
                point->x += velocities[i].x() * period;
                point->y += velocities[i].y() * period;
                point->z += velocities[i].z() * period;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%ld. Obstacle pos: (%f, %f, %f)", i, pos.x(), pos.y(), pos.z());
        }
    }
    clusters = obstacles;
}

// Reflect obstacles according to the principle of light reflecting
void perception_etflab::Obstacles::moveLightDirections(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster { nullptr };
    float tol_radius {}, t_param {};
    Eigen::Vector3f pos {}, pos_next {}, vec_normal {};
    bool change { true };

    for (size_t i = 0; i < num_rand_obstacles; i++)
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
