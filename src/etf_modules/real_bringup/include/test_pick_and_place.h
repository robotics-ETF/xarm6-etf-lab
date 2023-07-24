#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_ros_client.h>

#define c M_PI/180

using namespace std::chrono_literals;

class TestPickAndPlaceNode : public rclcpp::Node
{
public:
    TestPickAndPlaceNode();

private:
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<rclcpp::Node> xarm_client_node;
    xarm_api::XArmROSClient xarm_client;

    int period = 4;
    int state = 0;
    int k = 0;
    const std::vector<float> home_angles = {0, 0, 0, M_PI, M_PI_2, 0};
    std::vector<float> object_angles_approach = {90*c, 33*c, -96*c, 0*c, 63*c, 0*c};
    std::vector<float> current_angles;
    std::vector<float> current_pose;
    float delta_theta1 = 90*c;
    float object_height = 27;
    float object_pick_z = 50;
    
    const float max_lin_vel = 500;      // in [mm/s]
    const float max_lin_acc = 500;      // in [mm/s²]
    const float max_ang_vel = 1.5;      // in [rad/s]
    const float max_ang_acc = 1.5;      // in [rad/s²]

    void timerCallback();
};
