#include "pointcloud_combiner.h"

PointCloudCombiner::PointCloudCombiner() : Node("pointcloud_combiner")
{
	this->declare_parameter<std::string>("output_topic", "pointcloud");
	RCLCPP_INFO(this->get_logger(), "Starting up...");
	this->declare_parameter<std::vector<std::string>>("point_cloud_topics", std::vector<std::string>());
	this->declare_parameter<std::string>("base_frame", "world");
	this->get_parameter("point_cloud_topics", point_cloud_topics);
	this->get_parameter("base_frame", base_frame);
	this->get_parameter("output_topic", output_topic);
	
	RCLCPP_INFO(this->get_logger(), "Combined point cloud will be published on topic: %s", output_topic.c_str());
	
	publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 1);
			
	tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, true);

	for (auto point_cloud_topic : this->point_cloud_topics)
	{
		RCLCPP_INFO(this->get_logger(), "Adding subscription for point cloud at path: %s", point_cloud_topic.c_str());
		subscriptions.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
			point_cloud_topic, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
			std::bind(&PointCloudCombiner::pointCloudCallback, this, std::placeholders::_1)));
	}
}

void PointCloudCombiner::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	geometry_msgs::msg::TransformStamped transform_stamped_msg;
	std::string frame_id = msg->header.frame_id;
	try
	{
		transform_stamped_msg = tf_buffer->lookupTransform("world", frame_id, rclcpp::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		RCLCPP_WARN(this->get_logger(), "%s", ex.what());
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::moveFromROSMsg(*msg, *pcl_cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::transformPointCloud(*pcl_cloud, *pcl_transformed_cloud, tf2::transformToEigen(transform_stamped_msg).matrix());

	point_clouds[frame_id] = pcl_transformed_cloud;
	publishPoints();
}

void PointCloudCombiner::publishPoints()
{
	if (point_clouds.empty())
		return;

	pcl::PointCloud<pcl::PointXYZRGB> combined_pcl_cloud;
	for (const auto &points_pair : point_clouds)
		combined_pcl_cloud += *points_pair.second;

	sensor_msgs::msg::PointCloud2 combined_cloud;
	pcl::toROSMsg(combined_pcl_cloud, combined_cloud);

	combined_cloud.header.frame_id = base_frame;
	combined_cloud.header.stamp = now();

	publisher->publish(combined_cloud);
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PointCloudCombiner>());
	rclcpp::shutdown();
	return 0;
}
