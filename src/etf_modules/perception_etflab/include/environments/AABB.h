#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace perception_etflab
{
    class AABB
    {
    public:
        AABB() {}

        inline pcl::PointCloud<pcl::PointXYZ>::Ptr getBoxes() const { return boxes; }

		void make(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
        void publish();
		void visualize();

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr boxes;

    };
}