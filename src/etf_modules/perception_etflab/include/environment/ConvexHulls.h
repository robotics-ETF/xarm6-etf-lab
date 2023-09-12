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

#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>

namespace perception_etflab
{
    class ConvexHulls
    {
    public:
        ConvexHulls() {}

        inline const pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPoints() { return points; }
        inline const pcl::PointCloud<pcl::PointXYZ>::Ptr getPolygonsIndices() { return polygons_indices; }

		void make(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
        void publish();
		void visualize();

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr polygons_publisher;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;
        
    private:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
        pcl::PointCloud<pcl::PointXYZ>::Ptr polygons_indices;
    };
}