#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <fcl/fcl.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fcl/geometry/shape/convex.h>

#include <RBTConnect.h>
#include <RGBTConnect.h>
#include <RGBMTStar.h>
#include <xArm6.h>
#include <Scenario.h>
#include <RealVectorSpace.h>
#include <ConfigurationReader.h>
#include <glog/logging.h>

using namespace std::chrono_literals;

class TestPlannersNode : public rclcpp::Node
{
public:
    TestPlannersNode();

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_states_subscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr bounding_boxes_subscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr convex_hulls_subscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr convex_hulls_polygons_subscription;
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr octomap_client;

    const std::string scenario_file_path = "/sim_bringup/data/scenario_etf_lab.yaml";	
    std::string project_path;
    std::shared_ptr<scenario::Scenario> scenario;
    std::shared_ptr<robots::AbstractRobot> robot;
    std::shared_ptr<env::Environment> env;
    std::shared_ptr<base::State> start;
    std::shared_ptr<base::State> goal;
    std::vector<std::shared_ptr<base::State>> planner_path;
	std::vector<float> path_times;
	trajectory_msgs::msg::JointTrajectory trajectory;
    octomap::OcTree* octomap_octree;
	std::shared_ptr<fcl::OcTreef> octree;
    std::vector<fcl::Vector3f> bounding_boxes;
    std::vector<std::vector<fcl::Vector3f>> convex_hulls;
    std::vector<std::vector<int>> convex_hulls_polygons;
    std::shared_ptr<rclcpp::Node> read_octomap_node;
    int state;
    const int period = 1;               // in [s]
    const float max_ang_vel = 1.5;      // in [rad/s]

    void testPlannersCallback();
    void jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
    void boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void convexHullsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void convexHullsPolygonsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
	void readOctomap();
    void visualizeOctreeBoxes();
    void updateEnvironment();
    void parametrizePath(float max_ang_vel);
	void publishTrajectory(float init_time = 0);
	bool planPath();
    
};