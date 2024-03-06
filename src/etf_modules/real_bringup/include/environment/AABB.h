#include <rclcpp/rclcpp.hpp>
#include <fcl/fcl.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Environment.h>
#include <yaml-cpp/yaml.h>

namespace real_bringup
{
    class AABB
    {
    public:
        AABB(const std::string &config_file_path);

        inline const std::vector<Eigen::Vector3f> &getDimensions() { return dimensions; }
        inline const Eigen::Vector3f &getDimensions(int idx) { return dimensions[idx]; }
        inline const std::vector<Eigen::Vector3f> &getPositions() { return positions; }
        inline const Eigen::Vector3f &getPositions(int idx) { return positions[idx]; }
        inline const int getMinNumCaptures() { return min_num_captures; }
        inline void setEnvironment(const std::shared_ptr<env::Environment> &env_) { env = env_; }
        inline void setMinNumCaptures(int min_num_captures_) { min_num_captures = min_num_captures_; }

        void updateEnvironment();
        void resetMeasurements();
        virtual const int chooseObject() { return -1; }
        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void withFilteringCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

    protected:
        virtual bool whetherToRemove(Eigen::Vector3f &object_pos, Eigen::Vector3f &object_dim);

        std::vector<Eigen::Vector3f> dimensions;
        std::vector<Eigen::Vector3f> positions;
        std::vector<int> num_captures;
        int min_num_captures;
        std::shared_ptr<env::Environment> env;
    };
}