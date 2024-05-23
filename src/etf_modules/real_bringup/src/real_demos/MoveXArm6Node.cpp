#include "real_demos/MoveXArm6Node.h"

real_bringup::MoveXArm6Node::MoveXArm6Node(const std::string &node_name, const std::string &config_file_path) : 
    sim_bringup::MoveXArm6Node(node_name, config_file_path) 
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
    // Mode 1: External trajectory planner (position) mode (JointTrajectoryController can be used)
    // Mode 2: Free-Drive (zero gravity) mode.
    // Mode 3: Reserved.
    // Mode 4: Joint velocity control mode.
    // Mode 5: Cartesian velocity control mode.
    // Mode 6: Joint space online planning mode. (Firmware >= v1.10.0)
    // Mode 7: Cartesian space online planning mode. (Firmware >= v1.11.0)
    xarm_client.set_mode(0);
    xarm_client.set_state(0);

    xarm_client.set_gripper_enable(true);
    xarm_client.set_gripper_mode(0);
    xarm_client.set_gripper_speed(3000);

    set_position_node = std::make_shared<rclcpp::Node>("set_position_node");
    set_position_client = set_position_node->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
    
    for (float coord : Robot::getHomeJointsPosition())
        home_angles.emplace_back(coord);
        
    state = going_home;
}

void real_bringup::MoveXArm6Node::moveXArm6Callback()
{
    // testMode0();
    // testMode01();
    // testMode4();
    testMode6();
    // testGripper();
}

void real_bringup::MoveXArm6Node::testMode0()
{
    xarm_client.set_mode(0);
    xarm_client.set_state(0);

    switch (state)
    {
    case going_home:
        goHome();
        state = setting_position1;
        break;

    case setting_position1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting position 1...");
        xarm_client.set_position({500, 0, 133, 0, M_PI_2, 0}, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, false);
        // setPosition({500, 0, 133, 0, M_PI_2, 0}, 200, 1000);
        state = setting_position2;
        break;

    case setting_position2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting position 2...");
        xarm_client.set_position({400, 0, 133, 0, M_PI_2, 0}, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, false);
        // setPosition({400, 0, 133, 0, M_PI_2, 0}, 200, 1000);
        state = going_home;

    default:
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void real_bringup::MoveXArm6Node::testMode01()
{
    switch (state)
    {
    case going_home:
        goHome();
        state = moving_in_joint_space;
        break;

    case moving_in_joint_space:
        xarm_client.set_mode(1);
        xarm_client.set_state(0);
        moveInJointSpace();
        state = going_home;
        break;

    default:
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void real_bringup::MoveXArm6Node::testMode4()
{
    xarm_client.set_mode(4);
    xarm_client.set_state(0);
    xarm_client.set_joint_maxacc(10);   // maximal: 20 [rad/s²]

    switch (state)
    {
    case going_home:
        goHome();
        state = setting_position1;
        break;

    case setting_position1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting position 1...");
        xarm_client.vc_set_joint_velocity({-0.5, 0, 0, 0, 0, 0}, true, 1);
        state = setting_position2;
        break;

    case setting_position2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting position 2...");
        xarm_client.vc_set_joint_velocity({0.5, 0, 0, 0, 0, 0}, true, 1);
        state = setting_position1;
        break;
    
    default:
        break;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void real_bringup::MoveXArm6Node::testMode6()
{
    xarm_client.set_mode(6);
    xarm_client.set_state(0);

    switch (state)
    {
    case going_home:
        goHome();
        state = setting_position1;
        break;

    case setting_position1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting position 1...");
        xarm_client.set_servo_angle({-M_PI, 0, 0, M_PI, M_PI_2, 0}, 0.1, 0, 0, false);  // speed can be set from 0 to PI [rad/s]
        state = setting_position2;
        break;

    case setting_position2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting position 2...");
        xarm_client.set_servo_angle({-M_PI_2, -M_PI_4, 0, M_PI, M_PI_2, 0}, 0.5, 0, 0, false);
        state = setting_position1;
        break;

    default:
        break;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void real_bringup::MoveXArm6Node::testGripper()
{
    xarm_client.set_mode(0);
    xarm_client.set_state(0);

    switch (state)
    {
    case going_home:
        goHome();
        state = closing_gripper;
        break;

    case closing_gripper:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Closing gripper..."); 
        moveGripper(0);
        // xarm_client.set_gripper_position(0);
        state = opening_gripper;
        break;

    case opening_gripper:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening gripper..."); 
        moveGripper(1);
        // xarm_client.set_gripper_position(850, true, 1);
        state = going_home;
        break;

    default:
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

// Note: Mode will be switched to 0.
void real_bringup::MoveXArm6Node::goHome()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home...");
    xarm_client.set_mode(0);
    xarm_client.set_state(0);
    xarm_client.set_servo_angle(home_angles, Robot::getMaxVel(0), 0, 0, true);
    // xarm_client.move_gohome(true);

}

void real_bringup::MoveXArm6Node::moveInJointSpace()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving in joint space...");
    std::vector<Eigen::VectorXf> path {};
    std::vector<float> time_instances {};
    Eigen::VectorXf q(6);

    q << -M_PI_2, 0, 0, M_PI, M_PI_2, 0;
    path.emplace_back(q);
    time_instances.emplace_back(1);
    
    q << -M_PI_2, -M_PI_4, 0, M_PI, M_PI_2, 0;
    path.emplace_back(q);
    time_instances.emplace_back(2);

    q << -1, -5, -172, 180, 92, 0;
    q *= deg2rad;
    path.emplace_back(q);
    time_instances.emplace_back(5);

    Trajectory::clear();
    Trajectory::addPath(path, time_instances);
    Trajectory::publish();
}

void real_bringup::MoveXArm6Node::setPosition(const std::vector<float> &pose, float speed, float acceleration)
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
    
    auto request { std::make_shared<xarm_msgs::srv::MoveCartesian::Request>() };
    request->pose = pose;
    request->speed = speed;
    request->acc = acceleration;

    auto result { set_position_client->async_send_request(request) };
    if (rclcpp::spin_until_future_complete(set_position_node, result) == rclcpp::FutureReturnCode::SUCCESS)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ("Message: " + result.get()->message).c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set position!");
}

void real_bringup::MoveXArm6Node::testOrientation()
{
    std::vector<float> current_pose {};
    xarm_client.get_position(current_pose);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot end-effector pose: (%f, %f, %f)", current_pose[0], current_pose[1], current_pose[2]);     // XYZ in [m]
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot end-effector RPY:  (%f, %f, %f)", current_pose[3], current_pose[4], current_pose[5]);     // RPY angles in [rad]
    
    Eigen::Matrix3f R {};
    Eigen::Vector3f RPY {}, YPR {};

    // For approaching from above
    // R.col(0) << 0, 0, -1;
    // R.col(1) << -current_pose[1], current_pose[0], 0;
    // R.col(1) = R.col(1) / R.col(1).norm();
    // R.col(2) << current_pose[0], current_pose[1], 0;
    // R.col(2) = R.col(2) / R.col(2).norm();

    R = Eigen::AngleAxisf(current_pose[5], Eigen::Vector3f::UnitZ()) 
      * Eigen::AngleAxisf(current_pose[4], Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(current_pose[3], Eigen::Vector3f::UnitX());
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rotation matrix: ");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector n: (%f, %f, %f)", R(0,0), R(1,0), R(2,0));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector s: (%f, %f, %f)", R(0,1), R(1,1), R(2,1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector a: (%f, %f, %f)", R(0,2), R(1,2), R(2,2));

    YPR = R.eulerAngles(2, 1, 0);   // R = Rz(yaw) * Ry(pich) * Rx(roll)
    RPY << YPR(2), YPR(1), YPR(0);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RPY: (%f, %f, %f)", RPY(0), RPY(1), RPY(2));

    R = Eigen::AngleAxisf(RPY(2), Eigen::Vector3f::UnitZ()) 
      * Eigen::AngleAxisf(RPY(1), Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(RPY(0), Eigen::Vector3f::UnitX());
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rotation matrix: ");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector n: (%f, %f, %f)", R(0,0), R(1,0), R(2,0));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector s: (%f, %f, %f)", R(0,1), R(1,1), R(2,1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector a: (%f, %f, %f)", R(0,2), R(1,2), R(2,2));
}
