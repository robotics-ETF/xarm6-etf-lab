#include "real_demos/RealTimePlanningNode.h"

typedef planning::drbt::DRGBT DP;    // 'DP' is Dynamic Planner

real_bringup::RealTimePlanningNode::RealTimePlanningNode(const std::string &node_name, const std::string &config_file_path, 
                                                         const std::string &output_file_name) : 
    sim_bringup::RealTimePlanningNode(node_name, config_file_path, output_file_name)
{    
    publishing_trajectory_timer = this->create_wall_timer(std::chrono::microseconds(size_t(Trajectory::getTrajectoryMaxTimeStep() * 1e6)), 
                                  std::bind(&RealTimePlanningNode::publishingTrajectoryCallback, this));
    
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
    float t_delay { DP::updateCurrentState(true) };
    if (DP::spline_next == DP::spline_current)  // Trajectory has been already computed!
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Not computing a new trajectory! ");
        return;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New trajectory is computed! ");
    DP::spline_next->setTimeStart(t_delay);
}

void real_bringup::RealTimePlanningNode::publishingTrajectoryCallback()
{
    std::vector<float> position {};
    float t { DP::spline_next->getTimeCurrent(true) + Trajectory::getTrajectoryMaxTimeStep() };
    // std::cout << "Time: " << t << " [s]\t Position: ";
    for (size_t i = 0; i < Robot::getNumDOFs(); i++)
    {
        position.emplace_back(DP::spline_next->getPosition(t, i));
        // std::cout << position[i] << " ";
    }
    // std::cout << "\n";
    
    // xarm_client.set_servo_angle_j(position);                                    // When using mode 1
    xarm_client.set_servo_angle(position, Robot::getMaxVel(0), 0, 0, false);    // When using mode 6
}
