#include "base.h"

// 'period' is expressed in milliseconds by default.
BaseNode::BaseNode(const std::string node_name, const int period, const std::string time_unit) : Node(node_name)
{
    if (time_unit == "milliseconds")
        timer = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&BaseNode::baseCallback, this));
    else if (time_unit == "seconds")
        timer = this->create_wall_timer(std::chrono::seconds(period), std::bind(&BaseNode::baseCallback, this));
    else if (time_unit == "microseconds")
        timer = this->create_wall_timer(std::chrono::microseconds(period), std::bind(&BaseNode::baseCallback, this));
    else if (time_unit == "nanoseconds")
        timer = this->create_wall_timer(std::chrono::nanoseconds(period), std::bind(&BaseNode::baseCallback, this));
    
    trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>
        ("/xarm6_traj_controller/joint_trajectory", 10);
    gripper_node = std::make_shared<rclcpp::Node>("gripper_node");
    gripper_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(gripper_node, "/xarm_gripper/gripper_action");
    
    joint_states_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
        ("/xarm6_traj_controller/state", 10, std::bind(&BaseNode::jointStatesCallback, this, std::placeholders::_1));

    BaseNode::period = period;
    BaseNode::state = 0;
    joint_states = Eigen::VectorXf(6);
    trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}

void BaseNode::jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    std::vector<double> positions = msg->actual.positions;
	joint_states << positions[0], positions[1], positions[2], positions[3], positions[4], positions[5];
	// RCLCPP_INFO(this->get_logger(), "Robot joint states: (%f, %f, %f, %f, %f, %f).", 
    //     joint_states(0), joint_states(1), joint_states(2), joint_states(3), joint_states(4), joint_states(5));
}

void BaseNode::publishTrajectory(const std::vector<Eigen::VectorXf> path, const std::vector<float> path_times, float init_time)
{
    if (path.empty())
    {
        RCLCPP_INFO(this->get_logger(), "There is no trajectory to publish!\n");
        return;
    }
    
    trajectory.points.clear();

    RCLCPP_INFO(this->get_logger(), "Trajectory: ");
    for (int i = 0; i < path.size(); i++)
    {
        Eigen::VectorXf q = path[i];
        RCLCPP_INFO(this->get_logger(), "Num. %d. Time: %f [s]. Point: (%f, %f, %f, %f, %f, %f)", 
            i, path_times[i] + init_time, q(0), q(1), q(2), q(3), q(4), q(5));

        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (int j = 0; j < q.size(); j++)
            point.positions.emplace_back(q(j));

        point.time_from_start.sec = int32_t(path_times[i] + init_time);
        point.time_from_start.nanosec = (path_times[i] + init_time - point.time_from_start.sec) * 1e9;
        trajectory.points.emplace_back(point);
    }

    trajectory_publisher->publish(trajectory);
    RCLCPP_INFO(this->get_logger(), "Publishing trajectory ...\n");
}

void BaseNode::publishTrajectory(const std::vector<std::vector<float>> path, const std::vector<float> path_times, float init_time)
{
    if (path.empty())
    {
        RCLCPP_INFO(this->get_logger(), "There is no trajectory to publish!\n");
        return;
    }
    
    trajectory.points.clear();

    RCLCPP_INFO(this->get_logger(), "Trajectory: ");
    for (int i = 0; i < path.size(); i++)
    {
        std::vector<float> q = path[i];
        RCLCPP_INFO(this->get_logger(), "Num. %d. Time: %f [s]. Point: (%f, %f, %f, %f, %f, %f)", 
            i, path_times[i] + init_time, q[0], q[1], q[2], q[3], q[4], q[5]);

        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (int j = 0; j < q.size(); j++)
            point.positions.emplace_back(q[j]);

        point.time_from_start.sec = int32_t(path_times[i] + init_time);
        point.time_from_start.nanosec = (path_times[i] + init_time - point.time_from_start.sec) * 1e9;
        trajectory.points.emplace_back(point);
    }

    trajectory_publisher->publish(trajectory);
    RCLCPP_INFO(this->get_logger(), "Publishing trajectory ...\n");
}

// Close gripper: position = 0.0
// Open gripper: position = 1.0
void BaseNode::moveGripper(float position, float max_effort)
{
    if (!gripper_client->wait_for_action_server()) 
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting!");
      rclcpp::shutdown();
    }

    auto goal = control_msgs::action::GripperCommand::Goal();
    goal.command.position = 1.0 - position;  // Inverse logic
    goal.command.max_effort = max_effort;

    // RCLCPP_INFO(this->get_logger(), "Sending goal...");

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    
    send_goal_options.goal_response_callback = [this](auto goal_response) 
    { 
        auto goal_handle = goal_response.get();
        if (!goal_handle)
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server!");
        else
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result.");
    };

    send_goal_options.result_callback = [this](const auto &result) 
    {  
        switch (result.code) 
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted!");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled!");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code!");
            return;
        }

        std::stringstream ss;
        ss << "Result received. Position: " << result.result->position << "\tMax. effort: " << result.result->effort;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    };

    gripper_client->async_send_goal(goal, send_goal_options);
}
