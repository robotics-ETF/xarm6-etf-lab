#include "move_xarm6.h"

MoveXArm6Node::MoveXArm6Node(const std::string node_name, const int period, const std::string time_unit) 
    : BaseNode(node_name, period, time_unit) {}

void MoveXArm6Node::moveXArm6Callback()
{
    switch (state)
    {
    case 1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 1"); 
        moveInJointSpace();
        state++;
        break;

    default:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case default");
        goHome();
        state = 1;
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

// In order to use this function, please set xarm_client.set_mode(1)
void MoveXArm6Node::goHome()
{
    std::vector<std::vector<float>> path;
    std::vector<float> path_times;

    path.emplace_back(home_angles);
    path_times.emplace_back(0.9 * period);
    
    publishTrajectory(path, path_times, 0.5);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home...");
}

// In order to use this function, please set xarm_client.set_mode(1)
void MoveXArm6Node::moveInJointSpace()
{
    std::vector<Eigen::VectorXf> path;
    std::vector<float> path_times;
    Eigen::VectorXf q(6);

    q << -M_PI_2, 0, 0, M_PI, M_PI_2, 0;
    path.emplace_back(q);
    path_times.emplace_back(2);
    
    q << -M_PI_2, -M_PI_4, 0, M_PI, M_PI_2, 0;
    path.emplace_back(q);
    path_times.emplace_back(3);

    publishTrajectory(path, path_times);
}
