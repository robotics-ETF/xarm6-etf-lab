#include "demos/MoveXArm6Node2.h"

real_bringup::MoveXArm6Node2::MoveXArm6Node2(const std::string &node_name, const std::string &config_file_path) : 
    sim_bringup::moveXArm6Node(node_name, config_file_path) 
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
    
    state = going_home;
}

void real_bringup::MoveXArm6Node2::moveXArm6Callback2()
{
    switch (state)
    {
    case going_home:
        goHome();
        // state = moving_in_joint_space;
        state = setting_position1;
        break;

    case moving_in_joint_space:
        moveInJointSpace();
        state = going_home;
        break;

    case setting_position1:
        setPosition({500, 0, 133, 0, M_PI_2, 0}, 200, 1000);
        // xarm_client.set_position({500, 0, 133, 0, M_PI_2, 0}, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, false);
        state = setting_position2;
        break;

    case setting_position2:
        setPosition({400, 0, 133, 0, M_PI_2, 0}, 200, 1000);
        // xarm_client.set_position({400, 0, 133, 0, M_PI_2, 0}, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, false);
        state = closing_gripper;

    case closing_gripper:
        xarm_client.set_gripper_position(0);
        // moveGripper(0);
        state = opening_gripper;
        break;

    case opening_gripper:
        xarm_client.set_gripper_position(850, true, 1);
        // moveGripper(1);
        state = going_home;
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void real_bringup::MoveXArm6Node2::setPosition(const std::vector<float> &pose, float speed, float acceleration)
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
