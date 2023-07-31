#include "base.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <RRTConnect.h>
#include <RBTConnect.h>
#include <RGBTConnect.h>
#include <RGBMTStar.h>
#include <xArm6.h>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <RealVectorSpaceState.h>
#include <glog/logging.h>

class PlanningNode : public BaseNode
{
public:
    PlanningNode(const std::string scenario_file_path, const std::string node_name, const int period, 
                 const std::string time_unit = "milliseconds");

protected:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr bounding_boxes_subscription;
    
    std::string project_path;
    std::shared_ptr<scenario::Scenario> scenario;
    std::shared_ptr<robots::AbstractRobot> robot;
    std::shared_ptr<env::Environment> env;
    std::shared_ptr<base::State> start;
    std::shared_ptr<base::State> goal;
    std::vector<Eigen::VectorXf> path;
    std::vector<float> path_times;
    std::vector<std::shared_ptr<base::State>> planner_path;
    std::vector<fcl::Vector3f> bounding_boxes;

    virtual void baseCallback() override { planningCallback(); }
    virtual void planningCallback();
    void boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void updateEnvironment();
    void parametrizePath();
    void parametrizePath(float delta_time);
	bool planPath();
    
};
