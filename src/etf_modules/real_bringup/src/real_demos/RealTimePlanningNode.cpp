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
    xarm_client.set_mode(1);
    xarm_client.set_state(0);
    xarm_client.set_joint_maxacc(Robot::getMaxAcc(0));
    xarm_client.set_joint_jerk(Robot::getMaxJerk(0));
    xarm_client.save_conf();
}

void real_bringup::RealTimePlanningNode::computeTrajectory()
{
    // Only the following code is necessary, since trajectory is published in 'publishingTrajectoryCallback' function using 'xarm_client'.
    DP::visited_states = { DP::q_next };
    DP::updating_state->setNonZeroFinalVel(DP::q_next->getIsReached() && 
                                           DP::q_next->getIndex() != -1 && 
                                           DP::q_next->getStatus() != planning::drbt::HorizonState::Status::Goal);
    DP::updating_state->setTimeIterStart(DP::time_iter_start);
    DP::updating_state->update(DP::q_previous, DP::q_current, DP::q_next->getState(), DP::q_next->getStateReached(), DP::status);

    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    float t_wait { DP::updating_state->getWaitingTime() };
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting time: %f [us]", t_wait * 1e6);
    while (DP::getElapsedTime(time_start_) < t_wait) {}    // Wait for 't_wait' to exceed...
    time_traj_computed = std::chrono::steady_clock::now();
}

void real_bringup::RealTimePlanningNode::publishingTrajectoryCallback()
{
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inside publishingTrajectoryCallback...");
    std::vector<float> position(Robot::getNumDOFs());
    float t { getCurrTrajTime() + Trajectory::getTrajectoryMaxTimeStep() + trajectory_advance_time };
    Eigen::VectorXf pos { DP::traj->getPosition(t) };
    // if (Robot::getNumDOFs() == 6)
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time: %f [s]\t Position: (%f, %f, %f, %f, %f, %f)",
    //                                    t, pos(0), pos(1), pos(2), pos(3), pos(4), pos(5));
    for (size_t i = 0; i < Robot::getNumDOFs(); i++)
        position[i] = pos(i);
    
    xarm_client.set_servo_angle_j(position);  // When using mode 1
    // xarm_client.set_servo_angle(position, Robot::getMaxVel(0), 0, 0, false);    // When using mode 6
}
