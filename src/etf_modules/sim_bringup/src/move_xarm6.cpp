#include "move_xarm6.h"

MoveXArm6Node::MoveXArm6Node(const std::string node_name, const int period, const std::string time_unit) 
    : BaseNode(node_name, period, time_unit)
{   
    xarm_client_node = std::make_shared<rclcpp::Node>("xarm_client_node");
    xarm_client.init(xarm_client_node, "xarm");
    // xarm_client.clean_error();
    // xarm_client.clean_warn();
    // xarm_client.motion_enable(true);
    // xarm_client.set_mode(0);
    // xarm_client.set_state(0);
    // xarm_client.set_gripper_enable(true);
    // xarm_client.set_gripper_mode(0);
    // xarm_client.set_gripper_speed(2000);

    set_position_client = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
}

void MoveXArm6Node::moveXArm6Callback()
{
    switch (state)
    {
    case 1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 1"); 
        moveGripper(0.1);
        // setPosition({600, 0, 133, 0, M_PI_2, 0}, 300, 1000);
        state++;
        break;

    case 2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 2"); 
        moveInJointSpace();
        state++;
        break;

    default:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case default"); 
        goHome();
        moveGripper(1);
        state = 1;
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void MoveXArm6Node::goHome()
{
    std::vector<Eigen::VectorXf> path;
    std::vector<float> path_times;
    Eigen::VectorXf q(6);

    for (int i = 0; i < home_angles.size(); i++)
        q(i) = home_angles[i];

    path.emplace_back(q);
    path_times.emplace_back(period);
    publishTrajectory(path, path_times);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home...");
}

void MoveXArm6Node::moveInJointSpace()
{
    std::vector<Eigen::VectorXf> path;
    std::vector<float> path_times;
    Eigen::VectorXf q(6);

    q << M_PI_2, 0, 0, M_PI, M_PI_2, 0;
    path.emplace_back(q);
    path_times.emplace_back(2);
    
    q << M_PI_2, -M_PI_4, 0, M_PI, M_PI_2, 0;
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
    if (rclcpp::spin_until_future_complete(std::make_shared<rclcpp::Node>("set_position_node"), result) == rclcpp::FutureReturnCode::SUCCESS)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ("Message: " + result.get()->message).c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set position!");
}
