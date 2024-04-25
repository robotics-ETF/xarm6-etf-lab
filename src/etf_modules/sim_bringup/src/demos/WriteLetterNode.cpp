#include "demos/WriteLetterNode.h"

sim_bringup::WriteLetterNode::WriteLetterNode(const std::string &node_name, const std::string &config_file_path) : 
    BaseNode(node_name, config_file_path) 
{
    state = going_home;
}

void sim_bringup::WriteLetterNode::WriteLetterCallback()
{
    switch (state)
    {
    case going_home:
        goHome();
        state = moving_in_joint_space;
        break;

    case moving_in_joint_space:
        moveInJointSpace();
        state = going_home;
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void sim_bringup::WriteLetterNode::goHome()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home...");
    Trajectory::clear();
    Trajectory::addPoint(0.9 * period, Robot::getHomeJointsPosition());
    Trajectory::publish();
}

void sim_bringup::WriteLetterNode::moveInJointSpace()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving in joint space...");

    // const Eigen::Vector3f pos(1, 1, 0);

    // For approaching from above
    // KDL::Vector n(pos.x(), pos.y(), 0); n.Normalize();
    // KDL::Vector s(pos.y(), -pos.x(), 0); s.Normalize();
    // KDL::Vector a(0, 0, -1);
    // 

    // For approaching by side
    // KDL::Vector n(0, 0, -1);
    // KDL::Vector s(-pos.y(), pos.x(), 0); s.Normalize();
    // KDL::Vector a(pos.x(), pos.y(), 0); a.Normalize();
    // KDL::Rotation R(n, s, a);
    // KDL::Vector p(0.5, 0.5, 0.2);

    
    // std::shared_ptr<base::State> q_current = Robot::getJointsPositionPtr();
    // std::shared_ptr<base::State> q_final = Robot::getRobot()->computeInverseKinematics(R, p);

    // bool result = Planner::solve(q_current, q_final, 0.1*period);

    // if (result)
    // {
    //     Trajectory::clear();
    // //Trajectory::addPoint(0.8 * period, q_final->getCoord());
    //     Trajectory::addPath(Planner::getPath());
    //     Trajectory::publish();
    // }

    std::vector<float> time_instances;
    std::vector<Eigen::VectorXf> trajec;

    float phi = 0;
    Eigen::VectorXf curr;
    float time = 0;
    for (int i = 0; i < 1000; i++){
        curr = {cos(phi), sin(phi), 0.3};
        trajec.emplace_back(curr);
        time_instances.emplace(time);
        phi += 2.*M_PI/i;
        time += 0.1;
    }

    Trajectory::addPath(trajec, time_instances);

}
