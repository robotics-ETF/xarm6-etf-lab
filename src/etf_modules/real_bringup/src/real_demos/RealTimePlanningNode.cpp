#include "real_demos/RealTimePlanningNode.h"

typedef planning::drbt::DRGBT DP;    // 'DP' is Dynamic Planner

real_bringup::RealTimePlanningNode::RealTimePlanningNode(const std::string &node_name, const std::string &config_file_path, 
                                                         bool loop_execution_, const std::string &output_file_name) : 
    sim_bringup::RealTimePlanningNode(node_name, config_file_path, loop_execution_, output_file_name)
{    
    publishing_trajectory_timer = this->create_wall_timer(std::chrono::microseconds(size_t(Trajectory::getTrajectoryMaxTimeStep() * 1e6)), 
                                  std::bind(&RealTimePlanningNode::publishingTrajectoryCallback, this), callback_group);
    
    xarm_client_node = std::make_shared<rclcpp::Node>("xarm_client_node");
    xarm_client.init(xarm_client_node, "xarm");
    xarm_client.clean_error();
    xarm_client.clean_warn();
    xarm_client.motion_enable(true);
    xarm_client.set_mode(6);
    xarm_client.set_state(0);
    xarm_client.set_joint_maxacc(Robot::getMaxAcc(0));
    xarm_client.set_joint_jerk(Robot::getMaxJerk(0));
    xarm_client.save_conf();
}

void real_bringup::RealTimePlanningNode::computeTrajectory()
{
    // Only the following code is necessary, since trajectory is published in 'publishingTrajectoryCallback' function using 'xarm_client'.
    float t_delay { DP::updateCurrentState(true) };
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New trajectory is computed! Delay time: %f [ms]", t_delay * 1e3);
    if (DP::splines->spline_next != DP::splines->spline_current)  // New spline is computed
        DP::splines->spline_next->setTimeStart(t_delay);
}

void real_bringup::RealTimePlanningNode::publishingTrajectoryCallback()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inside publishingTrajectoryCallback...");
    std::vector<float> position {};
    float t { DP::splines->spline_next->getTimeCurrent(true) + Trajectory::getTrajectoryMaxTimeStep() };
    Eigen::VectorXf pos { DP::splines->spline_next->getPosition(t) };
    // std::cout << "Time: " << t << " [s]\t Position: " << pos.transpose() << "\n";
    for (size_t i = 0; i < Robot::getNumDOFs(); i++)
        position.emplace_back(pos(i));
    
    // xarm_client.set_servo_angle_j(position);                                    // When using mode 1
    xarm_client.set_servo_angle(position, Robot::getMaxVel(0), 0, 0, false);    // When using mode 6
}
