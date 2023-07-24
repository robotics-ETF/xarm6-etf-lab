#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

class TestMoveXarm6Node : public rclcpp::Node
{
public:
    TestMoveXarm6Node();

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;

    void testPlannersCallback();
};
