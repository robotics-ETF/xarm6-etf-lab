#include "move_xarm6.h"

MoveXArm6Node::MoveXArm6Node(const std::string node_name, const int period, const std::string time_unit) 
    : BaseNode(node_name, period, time_unit)
{   
    // The defined service can only be activated at initialization if that service is configured to true. 
    // If you need to customize the parameters, please create a file xarm_api/config/xarm_user_params.yaml 
    // To modify the format, refer to xarm_api/config/xarm_params.yaml
    // See 5.4 xarm_api at: https://github.com/xArm-Developer/xarm_ros2/tree/humble
    xarm_client_node = std::make_shared<rclcpp::Node>("xarm_client_node");
    xarm_client.init(xarm_client_node, "xarm");
    xarm_client.clean_error();
    xarm_client.clean_warn();
    xarm_client.motion_enable(true);

    // See 6.1 Mode Explanation at: https://github.com/xArm-Developer/xarm_ros#report_type-argument
    // Mode 0: xArm controller (position) mode
    // Mode 1: External trajectory planner (position) mode
    xarm_client.set_mode(0);
    xarm_client.set_state(0);

    xarm_client.set_gripper_enable(true);
    xarm_client.set_gripper_mode(0);
    xarm_client.set_gripper_speed(3000);

    set_position_node = std::make_shared<rclcpp::Node>("set_position_node");
    set_position_client = set_position_node->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
}

void MoveXArm6Node::moveXArm6Callback()
{
    switch (state)
    {
    case 1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 1"); 
        setPosition({500, 0, 133, 0, M_PI_2, 0}, 200, 1000);
        // xarm_client.set_position({500, 0, 133, 0, M_PI_2, 0}, -1, 0.7*max_lin_vel, 0.7*max_lin_acc, 0, false);
        xarm_client.set_gripper_position(0);
        // moveGripper(0);
        state++;
        break;

    // case 2:
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 2");
    //     moveInJointSpace();
    //     state++;
    //     break;

    default:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case default");
        // goHome();
        setPosition({400, 0, 133, 0, M_PI_2, 0}, 200, 1000);
        xarm_client.set_gripper_position(850, true, 1);
        // moveGripper(1);
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

void MoveXArm6Node::setPosition(const std::vector<float> &pose, float speed, float acceleration)
{
    while (!set_position_client->wait_for_service(1s))
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service is ready!");
    
    auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
    request->pose = pose;
    request->speed = speed;
    request->acc = acceleration;

    auto result = set_position_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(set_position_node, result) == rclcpp::FutureReturnCode::SUCCESS)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ("Message: " + result.get()->message).c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set position!");
}
