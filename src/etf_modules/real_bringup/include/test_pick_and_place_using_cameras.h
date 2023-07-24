#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_ros_client.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class TestPickAndPlaceUsingCamerasNode : public rclcpp::Node
{
public:
    TestPickAndPlaceUsingCamerasNode();
    
private:
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<rclcpp::Node> xarm_client_node;
    xarm_api::XArmROSClient xarm_client;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr bounding_boxes_subscription;
    
    int period = 5;
    int state = 0;
    const float c = M_PI/180;
    const std::vector<float> home_angles = {0, 0, 0, M_PI, M_PI_2, 0};
    std::vector<float> current_angles;
    std::vector<float> current_pose;
    std::vector<float> object_approach_pose;
    std::vector<float> object_pick_pose;
    std::vector<Eigen::Vector3f> objects_pos;
    std::vector<Eigen::Vector3f> objects_dim;
    std::vector<int> num_captures;
    int max_num_captures = 10;
    int obj_idx;
    float delta_z = 120;
    float offset_z = 10;
    float gripper_pos;
    float theta_obj;

    const float max_lin_vel = 500;      // in [mm/s]
    const float max_lin_acc = 500;      // in [mm/s²]
    const float max_ang_vel = 1.5;      // in [rad/s]
    const float max_ang_acc = 1.5;      // in [rad/s²]

    void boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void testRobotOrientation();
    void chooseObject();
    void computeObjectApproachAndPickPose();
    void timerCallback();
};
