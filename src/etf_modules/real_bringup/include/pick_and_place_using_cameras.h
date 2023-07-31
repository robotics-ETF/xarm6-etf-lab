#include "pick_and_place.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class PickAndPlaceUsingCamerasNode : public PickAndPlaceNode
{
public:
    PickAndPlaceUsingCamerasNode(const std::string node_name, const int period, const std::string time_unit = "milliseconds");
    
protected:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr bounding_boxes_subscription;

    std::vector<Eigen::Vector3f> objects_pos;
    std::vector<Eigen::Vector3f> objects_dim;
    std::vector<int> num_captures;
    const int max_num_captures = 10;

    void boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void testRobotOrientation();
    void chooseObject();
    void computeObjectApproachAndPickPose();
    void pickAndPlaceCallback() override { pickAndPlaceUsingCamerasCallback(); };
    void pickAndPlaceUsingCamerasCallback();
    
private:
    std::vector<float> object_approach_pose;
    std::vector<float> object_pick_pose;
    int obj_idx;
    float delta_z = 120;
    float offset_z = 10;
    float gripper_pos;
    float theta_obj;

};
