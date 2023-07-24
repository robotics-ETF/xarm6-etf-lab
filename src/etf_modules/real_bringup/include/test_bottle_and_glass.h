#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_ros_client.h>

#define c M_PI/180

using namespace std::chrono_literals;

class TestBottleAndGlassNode : public rclcpp::Node
{
public:
    TestBottleAndGlassNode();

private:
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<rclcpp::Node> xarm_client_node;
    xarm_api::XArmROSClient xarm_client;

    int period = 4;
    int state = 0;
    int sign = 1;
    const std::vector<float> home_angles = {0, 0, 0, M_PI, M_PI_2, 0};
    std::vector<float> glass_angles_approach = {55*c, 35*c, -36*c, 180*c, 89*c, 0*c};
    std::vector<float> glass_angles_pick = {55*c, 41*c, -52*c, 180*c, 79*c, 0*c};
    std::vector<float> bottle_pose_pick = {640, 0, 90, 0, M_PI_2, 0};
    std::vector<float> current_angles;
    std::vector<float> current_pose;
    
    const float max_lin_vel = 500;      // in [mm/s]
    const float max_lin_acc = 500;      // in [mm/s²]
    const float max_ang_vel = 1.3;      // in [rad/s]
    const float max_ang_acc = 1.3;      // in [rad/s²]

    void timerCallback();
};
