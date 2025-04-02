#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace perception_etflab
{
    class Obstacles
    {
    public:
        Obstacles(const std::string &config_file_path);

        bool isValid(const Eigen::Vector3f &pos, float vel);
        void move(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
        void move(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);
        inline float getPeriod() const { return period; }

    private:
        void moveCircular(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
        void moveTwoTunnels(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
        void moveRandomDirections(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
        void moveLightDirections(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);

        size_t num_obstacles;
        size_t num_rand_obstacles;
        Eigen::Vector3f dim_rand;
		float robot_max_vel;                                            // Maximal velocity for each obstacle
        Eigen::Vector3f WS_center;								        // Workspace center point in [m]
        float WS_radius; 										        // Workspace radius in [m]
		float base_radius;
		size_t ground_included;
        float period; 						                            // Period in [s]
        float max_vel;                                                  // Maximal velocity of each obstacle in [m/s]
		std::vector<Eigen::Vector3f> velocities; 		                // Velocity vector for each obstacle
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> obstacles;  // Each obstacle represents a single cluster
        Eigen::VectorXf path_len;
        Eigen::VectorXi sign;

        enum class MotionType 
		{
			circular, 
			two_tunnels, 
			random_directions, 
			light_directions 
		}
		motion_type;
    };
}