#include "base/Robot.h"

real_bringup::Robot::Robot(const std::string config_file_path)
{
    std::string project_abs_path = std::string(__FILE__);
    for (int i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node = YAML::LoadFile(project_abs_path + config_file_path);
    YAML::Node robot_node = node["robot"];

    max_lin_vel = robot_node["max_lin_vel"].as<float>();
    max_lin_acc = robot_node["max_lin_acc"].as<float>();
    max_ang_vel = robot_node["max_ang_vel"].as<float>();
    max_ang_acc = robot_node["max_ang_acc"].as<float>();

    Eigen::VectorXf home_joints_coord(robot_node["home"].size());
    for (int i = 0; i < home_joints_coord.size(); i++)
        home_joints_coord(i) = robot_node["home"][i].as<float>();
    home_joints_state = std::make_shared<base::RealVectorSpaceState>(home_joints_coord);

    joints_state = nullptr;
}

void real_bringup::Robot::jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
	std::vector<double> positions = msg->actual.positions;
	Eigen::VectorXf q(6);
	q << positions[0], positions[1], positions[2], positions[3], positions[4], positions[5];
    joints_state = std::make_shared<base::RealVectorSpaceState>(q);
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot joint states: (%f, %f, %f, %f, %f, %f).", q(0), q(1), q(2), q(3), q(4), q(5));
}

bool real_bringup::Robot::isReady()
{
    if (joints_state == nullptr)
        return false;

    return true;
}

bool real_bringup::Robot::isReached(std::shared_ptr<base::State> q, float tol)
{
    if ((joints_state->getCoord() - q->getCoord()).norm() < tol)
        return true;

    return false;
}

// Close gripper: position = 0.0
// Open gripper: position = 1.0
void real_bringup::Robot::moveGripper(float position, float max_effort)
{
    if (!gripper_client->wait_for_action_server()) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Action server not available after waiting!");
      rclcpp::shutdown();
    }

    auto goal = control_msgs::action::GripperCommand::Goal();
    goal.command.position = 1.0 - position;  // Inverse logic
    goal.command.max_effort = max_effort;

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending goal...");

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    
    send_goal_options.goal_response_callback = [this](auto goal_response) 
    { 
        auto goal_handle = goal_response.get();
        if (!goal_handle)
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by server!");
        else
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by server, waiting for result.");
    };

    send_goal_options.result_callback = [this](const auto &result) 
    {  
        switch (result.code) 
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted!");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was canceled!");
            return;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code!");
            return;
        }

        std::stringstream ss;
        ss << "Result received. Position: " << result.result->position << "\tMax. effort: " << result.result->effort;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
    };

    gripper_client->async_send_goal(goal, send_goal_options);
}

void real_bringup::Robot::testOrientation()
{
    std::vector<float> current_pose;
    xarm_client.get_position(current_pose);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot end-effector pose: (%f, %f, %f)", current_pose[0], current_pose[1], current_pose[2]);     // XYZ in [m]
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot end-effector RPY:  (%f, %f, %f)", current_pose[3], current_pose[4], current_pose[5]);     // RPY angles in [rad]
    
    Eigen::Matrix3f R;
    Eigen::Vector3f RPY, YPR;

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