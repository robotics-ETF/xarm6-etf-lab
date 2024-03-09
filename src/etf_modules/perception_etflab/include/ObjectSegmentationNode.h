#include "Robot.h"
#include "environments/Clusters.h"
#include "environments/AABB.h"
#include "environments/ConvexHulls.h"
#include "environments/Obstacles.h"

using namespace std::chrono_literals;

namespace perception_etflab
{
	class ObjectSegmentationNode : public rclcpp::Node,
							   	   public perception_etflab::Robot,
								   public perception_etflab::Clusters,
								   public perception_etflab::AABB,
								   public perception_etflab::ConvexHulls,
								   public perception_etflab::Obstacles
	{
	public:
		ObjectSegmentationNode(const std::string &node_name, const std::string &config_file_path);

	protected:
		std::string input_cloud;
		std::string objects_cloud;
		bool real_robot;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pcl_publisher;

		void realPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		void simPointCloudCallback();
		void publishObjectsPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
		void removeOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);
	};
}