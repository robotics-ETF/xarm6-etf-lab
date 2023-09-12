#include <rclcpp/rclcpp.hpp>
#include <fcl/fcl.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <yaml-cpp/yaml.h>

namespace sim_bringup
{
    class ConvexHulls
    {
    public:
        ConvexHulls(const std::string config_file_path);

        inline const std::vector<std::vector<fcl::Vector3f>> &getPoints() { return points; }
        inline const std::vector<std::vector<int>> &getPolygons() { return polygons; }

        void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void polygonsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscription;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr polygons_subscription;

    private:
        std::vector<std::vector<fcl::Vector3f>> points;
        std::vector<std::vector<int>> polygons;
    };
}