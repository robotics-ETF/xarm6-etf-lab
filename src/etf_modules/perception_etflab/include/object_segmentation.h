#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <xarm_api/xarm_ros_client.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>

#include <xArm6.h>
#include <RealVectorSpace.h>
#include <yaml-cpp/yaml.h>
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/parse.h"

using namespace std::chrono_literals;

class ObjectSegmentation : public rclcpp::Node
{
public:
	ObjectSegmentation();

private:
	// const std::string robot_config_file_path = "/sim_bringup/data/scenario_etf_lab.yaml";
	const std::string robot_config_file_path = "/real_bringup/data/scenario_etf_lab.yaml";
	std::shared_ptr<robots::AbstractRobot> robot;
	std::shared_ptr<Eigen::MatrixXf> skeleton;
	std::shared_ptr<base::State> joint_states;
	std::string input_cloud;
	std::string objects_cloud;
    std::shared_ptr<rclcpp::Node> xarm_client_node;
    xarm_api::XArmROSClient xarm_client;

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription;
	rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_states_subscription;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_point_cloud_publisher;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bounding_boxes_publisher;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr convex_hulls_publisher;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr convex_hulls_polygons_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_free_cells_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_occupied_cells_publisher;

	void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
	void jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
	void publishObjectsPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
	void publishBoundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
	void publishConvexHulls(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
	void computeClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
	void computeSubclusters(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters,
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &subclusters, const Eigen::Vector3f &max_dim = Eigen::Vector3f(0.1, 0.1, 0.1));
	void divideCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &subclusters, 
		float min_point, float max_point, float max_dim, std::string &axis, const float tolerance = 0.05);
	void removeOutliers(pcl::PointCloud<pcl::PointXYZRGB> &pcl);
	void removeClustersOccupiedByRobot(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters, int tolerance_factor = 1.5);
	void removeClustersOccupiedByRobot_v2(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters, int tolerance_factor = 1.5);
	void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);
	void visualizeBoundingBoxes(pcl::PointCloud<pcl::PointXYZ>::Ptr bounding_boxes);
	void visualizeConvexHulls(pcl::PointCloud<pcl::PointXYZRGB>::Ptr convex_hulls_points);
	void visualizeRobotCapsules();
    void visualizeRobotSkeleton();

};
