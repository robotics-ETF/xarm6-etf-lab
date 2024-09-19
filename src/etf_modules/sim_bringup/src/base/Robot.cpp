#include "base/Robot.h"

sim_bringup::Robot::Robot(const std::string &config_file_path)
{
    std::string project_abs_path(__FILE__);
    for (size_t i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    try
    {
        YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
        YAML::Node robot_node { node["robot"] };

        num_DOFs = robot_node["num_DOFs"].as<size_t>();

        YAML::Node q_home_node { robot_node["q_home"] };
        if (q_home_node.size() != num_DOFs)
            throw std::logic_error("Number of home joint coordinates is not correct!");

        home_joints_position = Eigen::VectorXf(num_DOFs);
        for (size_t i = 0; i < num_DOFs; i++)
            home_joints_position(i) = q_home_node[i].as<float>();

        YAML::Node gripper_length_node { robot_node["gripper_length"] };
        float gripper_length { 0 };
        if (gripper_length_node.IsDefined())
            gripper_length = gripper_length_node.as<float>();
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Gripper is not included!");

        YAML::Node ground_included_node { robot_node["ground_included"] };
        size_t ground_included { 0 }; 
        if (ground_included_node.IsDefined()) 
            ground_included = ground_included_node.as<size_t>();
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Ground is not included in the scene!");

        std::string type { robot_node["type"].as<std::string>() };
        if (type == "xarm6")
            robot = std::make_shared<robots::xArm6>(project_abs_path + robot_node["urdf"].as<std::string>(),
                                                    gripper_length,
                                                    ground_included);
        else
            throw std::logic_error("Robot type is not specified!");

        YAML::Node capsules_radius_node { robot_node["capsules_radius"] };
        if (capsules_radius_node.IsDefined())
        {
            if (capsules_radius_node.size() != num_DOFs)
                throw std::logic_error("Number of capsules is not correct!");
                
            std::vector<float> capsules_radius {};
            for (size_t i = 0; i < num_DOFs; i++)
                capsules_radius.emplace_back(capsules_radius_node[i].as<float>());

            robot->setCapsulesRadius(capsules_radius);
        }
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Radii of robot's capsules are not defined! Considering all zeros.");

        YAML::Node max_vel_node { robot_node["max_vel"] };
        if (max_vel_node.IsDefined())
        {
            if (max_vel_node.size() != num_DOFs)
                throw std::logic_error("The size of 'max_vel' is not correct!");

            Eigen::VectorXf max_vel(num_DOFs);
            for (size_t i = 0; i < num_DOFs; i++)
                max_vel(i) = max_vel_node[i].as<float>();

            robot->setMaxVel(max_vel);
        }
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Maximal robot's joint velocity is not defined!");

        YAML::Node max_acc_node { robot_node["max_acc"] };
        if (max_acc_node.IsDefined())
        {
            if (max_acc_node.size() != num_DOFs)
                throw std::logic_error("The size of 'max_acc' is not correct!");

            Eigen::VectorXf max_acc(num_DOFs);
            for (size_t i = 0; i < num_DOFs; i++)
                max_acc(i) = max_acc_node[i].as<float>();

            robot->setMaxAcc(max_acc);
        }
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Maximal robot's joint acceleration is not defined!");

        YAML::Node max_jerk_node { robot_node["max_jerk"] };
        if (max_jerk_node.IsDefined())
        {
            if (max_jerk_node.size() != num_DOFs)
                throw std::logic_error("The size of 'max_jerk' is not correct!");

            Eigen::VectorXf max_jerk(num_DOFs);
            for (size_t i = 0; i < num_DOFs; i++)
                max_jerk(i) = max_jerk_node[i].as<float>();
                
            robot->setMaxJerk(max_jerk);
        }
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Maximal robot's joint jerk is not defined!");

        YAML::Node max_lin_vel_node { robot_node["max_lin_vel"] };
        if (max_lin_vel_node.IsDefined())
            max_lin_vel = max_lin_vel_node.as<float>();
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Maximal robot's linear velocity is not defined!");

        YAML::Node max_lin_acc_node { robot_node["max_lin_acc"] };
        if (max_lin_acc_node.IsDefined())
            max_lin_acc = max_lin_acc_node.as<float>();
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Maximal robot's linear acceleration is not defined!");

        YAML::Node max_lin_jerk_node { robot_node["max_lin_jerk"] };
        if (max_lin_jerk_node.IsDefined())
            max_lin_jerk = max_lin_jerk_node.as<float>();
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Maximal robot's linear jerk is not defined!");

        YAML::Node self_collision_checking_node { robot_node["self_collision_checking"] };
        if (self_collision_checking_node.IsDefined())
            robot->setSelfCollisionChecking(self_collision_checking_node.as<bool>());
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "It is not defined whether self-collision should be checked! By default, it is true.");
    
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << "\n";
    }

    joints_position = Eigen::VectorXf(num_DOFs);
    joints_velocity = Eigen::VectorXf(num_DOFs);
    joints_acceleration = Eigen::VectorXf(num_DOFs);
    ready = false;
}

void sim_bringup::Robot::jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    ready = false;
    for (size_t i = 0; i < num_DOFs; i++)
    {
        joints_position(i) = msg->actual.positions[i];
        joints_velocity(i) = msg->actual.velocities[i];
        // joints_acceleration(i) = msg->actual.accelerations[i];   // Not supported for xarm6.
    }
	ready = true;
    
    // if (num_DOFs == 6)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot joints position:     (%f, %f, %f, %f, %f, %f).", 
    //         joints_position(0), joints_position(1), joints_position(2), joints_position(3), joints_position(4), joints_position(5));
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot joints velocity:     (%f, %f, %f, %f, %f, %f).", 
    //         joints_velocity(0), joints_velocity(1), joints_velocity(2), joints_velocity(3), joints_velocity(4), joints_velocity(5));
    //     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot joints acceleration: (%f, %f, %f, %f, %f, %f).", 
    //     //     joints_acceleration(0), joints_acceleration(1), joints_acceleration(2), joints_acceleration(3), joints_acceleration(4), joints_acceleration(5));
    // }
}

bool sim_bringup::Robot::isReached(std::shared_ptr<base::State> q, float tol)
{
    if ((joints_position - q->getCoord()).norm() < tol)
        return true;

    return false;
}

// Close gripper: position = 0.0
// Open gripper: position = 1.0
void sim_bringup::Robot::moveGripper(float position, float max_effort)
{
    if (!gripper_client->wait_for_action_server()) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Action server not available after waiting!");
        return;
    }

    auto goal { control_msgs::action::GripperCommand::Goal() };
    goal.command.position = 1.0 - position;  // Inverse logic
    goal.command.max_effort = max_effort;

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending goal...");

    auto send_goal_options { rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions() };
    
    send_goal_options.goal_response_callback = [this](auto goal_response) 
    { 
        auto goal_handle { goal_response.get() };
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

        std::stringstream ss {};
        ss << "Result received. Position: " << result.result->position << "\tMax. effort: " << result.result->effort;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
    };

    gripper_client->async_send_goal(goal, send_goal_options);
}
