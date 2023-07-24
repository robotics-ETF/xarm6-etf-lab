#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "pcl/common/transforms.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class PointCloudCombiner : public rclcpp::Node
{
public:
    PointCloudCombiner();

private:
	std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
	std::vector<std::string> point_cloud_topics;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
	std::string base_frame;
	std::string output_topic;
	std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> point_clouds;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishPoints();
};