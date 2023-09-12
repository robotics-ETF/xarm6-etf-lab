#include "PointCloudCombinerNode.h"

int main(int argc, char *argv[])
{
	const std::string node_name = "pointcloud_combiner_node";

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<perception_etflab::PointCloudCombinerNode>(node_name));
	rclcpp::shutdown();
	return 0;
}