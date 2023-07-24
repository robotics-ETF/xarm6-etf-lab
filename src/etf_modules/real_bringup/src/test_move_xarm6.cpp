#include "test_move_xarm6.h"

TestMoveXarm6Node::TestMoveXarm6Node() : Node("test_move_xarm6_node")
{
    timer = this->create_wall_timer(std::chrono::seconds(period), std::bind(&TestMoveXarm6Node::timerCallback, this));
    trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
    // gripper_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm_gripper_traj_controller/joint_trajectory", 10);
    gripper_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(this, "/xarm_gripper/gripper_action");
    // move_cartesian_client = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_servo_cartesian");
    joint_states_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
        ("/xarm6_traj_controller/state", 10, std::bind(&TestMoveXarm6Node::jointStatesCallback, this, std::placeholders::_1));
    
    trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    // move_cartesian_node = std::make_shared<rclcpp::Node>("move_cartesian_node");
}

void TestMoveXarm6Node::goHome()
{
    trajectory.points.clear();
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0, 0, 0, M_PI, M_PI_2, 0};
    point.time_from_start.sec = period;
    trajectory.points.emplace_back(point);
    trajectory_publisher->publish(trajectory);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home..."); 
}

// Close gripper: position = 0.1
// Open gripper: position = 1
void TestMoveXarm6Node::moveGripper(float position, float max_effort)
{
    // trajectory.points.clear();
    // trajectory_msgs::msg::JointTrajectoryPoint point;
    // point.positions = {position};
    // point.effort = {max_effort};
    // point.time_from_start.sec = period;
    // gripper_publisher->publish(trajectory);

    control_msgs::action::GripperCommand::Goal gripper_command;
    control_msgs::msg::GripperCommand command;
    command.position = 1.0 - position;  // Inverse logic
    command.max_effort = max_effort;
    gripper_command.command = command;
    gripper_client->async_send_goal(gripper_command);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending gripper commad... Position: %f. Max. effort: %f", position, max_effort); 
}

void TestMoveXarm6Node::move1()
{
    trajectory.points.clear();
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {M_PI_2, 0, 0, M_PI, M_PI_2, 0};
    point.time_from_start.sec = 2;
    trajectory.points.emplace_back(point);

    point.positions = {M_PI_2, -M_PI_4, 0, M_PI, M_PI_2, 0};
    point.time_from_start.sec = 3;
    trajectory.points.emplace_back(point);

    trajectory_publisher->publish(trajectory);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing trajectory..."); 
}

void TestMoveXarm6Node::moveCartesian(const std::vector<float> &pose, float speed, float acceleration)
{
    while (!move_cartesian_client->wait_for_service(1s))
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

    auto result = move_cartesian_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(move_cartesian_node, result) == rclcpp::FutureReturnCode::SUCCESS)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ("Message: " + result.get()->message).c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to move cartesian!");
}

void TestMoveXarm6Node::timerCallback()
{
    switch (state)
    {
    case 1:
        move1();
        state++;
        break;

    case 2:
        moveGripper(0.5);
        state++;
        break;

    default:
        goHome();
        moveGripper(1);
        state = 1;
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void TestMoveXarm6Node::jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    std::vector<double> positions = msg->actual.positions;
    // RCLCPP_INFO(this->get_logger(), "Robot joint states: (%f, %f, %f, %f, %f, %f).", 
    //     positions[0], positions[1], positions[2], positions[3], positions[4], positions[5]);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestMoveXarm6Node>());
    rclcpp::shutdown();
    return 0;
}