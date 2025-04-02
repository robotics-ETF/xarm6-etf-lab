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
    DP::updating_state->setNonZeroFinalVel(DP::q_next->getIsReached() && DP::q_next->getIndex() != -1 && 
                                           DP::q_next->getStatus() != planning::drbt::HorizonState::Status::Goal);
    DP::updating_state->setTimeIterStart(DP::time_iter_start);
    DP::updating_state->setNextState(DP::q_next->getState());
    DP::updating_state->setMeasureTime(true);
    std::shared_ptr<base::State> q_next_reached { DP::q_next->getStateReached() };
    DP::updating_state->update(DP::q_previous, DP::q_current, q_next_reached, DP::status);

    float t_delay { DP::updating_state->getRemainingTime() };
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New trajectory is computed! Delay time: %f [ms]", t_delay * 1e3);
    if (DP::splines->spline_next != DP::splines->spline_current)  // New spline is computed
        DP::splines->spline_next->setTimeStart(t_delay);
}

void real_bringup::RealTimePlanningNode::publishingTrajectoryCallback()
{
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inside publishingTrajectoryCallback...");
    std::vector<float> position(Robot::getNumDOFs());
    float t { DP::splines->spline_next->getTimeCurrent(true) + Trajectory::getTrajectoryMaxTimeStep() + trajectory_advance_time };
    Eigen::VectorXf pos { DP::splines->spline_next->getPosition(t) };
    // if (Robot::getNumDOFs() == 6)
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time: %f [s]\t Position: (%f, %f, %f, %f, %f, %f)",
    //                                    t, pos(0), pos(1), pos(2), pos(3), pos(4), pos(5));
    for (size_t i = 0; i < Robot::getNumDOFs(); i++)
        position[i] = pos(i);
    
    xarm_client.set_servo_angle_j(position);  // When using mode 1
    // xarm_client.set_servo_angle(position, Robot::getMaxVel(0), 0, 0, false);    // When using mode 6
}
