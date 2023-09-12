#include <rclcpp/rclcpp.hpp>
#include <fcl/fcl.h>
#include <octomap/octomap.h>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <yaml-cpp/yaml.h>

namespace sim_bringup
{
    class Octomap
    {
    public:
        Octomap(const std::string config_file_path);

        inline const std::shared_ptr<fcl::OcTreef> &getOctree() { return octree; }

        void read();
        void visualize();

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;    

    private:
        std::shared_ptr<rclcpp::Node> read_node;
        rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client;
        octomap::OcTree* octomap_octree;
        std::shared_ptr<fcl::OcTreef> octree;
    };
}