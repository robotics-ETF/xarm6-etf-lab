#include "demos/DrawNode.h"


std::vector<double> x, y;
sim_bringup::DrawNode::DrawNode(const std::string &node_name, const std::string &config_file_path) : 
    BaseNode(node_name, config_file_path) 
{
    std::string project_abs_path(__FILE__);
    for (size_t i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
    YAML::Node dmp { YAML::LoadFile(project_abs_path + "/sim_bringup/dmps/dmp.yaml") };
    // YAML::Node dmp { YAML::LoadFile("/home/hanka/Desktop/DMP/dmp.yaml") };

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reading YAML file");

    YAML::Node node { dmp["x"] }; 
    if (!node.IsDefined()){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "no x coordinates");
        throw std::logic_error("failed to load file");
    }

    YAML::Node node2 { dmp["y"] };
    if (!node2.IsDefined()){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "no y coordinates");
        throw std::logic_error("failed to load file");
    }
    for (const auto& value : dmp["x"])
        x.push_back(value.as<double>()); 
    for (const auto& value : dmp["y"])
        y.push_back(value.as<double>()); 

    state = going_home;
}

void sim_bringup::DrawNode::DrawCallback()
{
    switch (state)
    {
        case going_home:
            goHome();
            if (Robot::isReady())
                state = moving_in_joint_space;
            break;

        case moving_in_joint_space:
            moveInJointSpace();
            state = going_home;
            break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void sim_bringup::DrawNode::goHome()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home...");
    Trajectory::clear();
    Trajectory::addPoint(0.5 * period, Robot::getHomeJointsPosition());
    Trajectory::publish();
}

bool sim_bringup::DrawNode::moveInJointSpace()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Drawing...");

    std::vector<float> time_instances {};
    float T = period*0.5;
    int instances = x.size();

    std::shared_ptr<base::State> q = { Robot::getJointsPositionPtr() }, prev_q;
    prev_q = q;

    std::vector<std::shared_ptr<base::State>> path;
    int k = 0;
    for (int i = 0; i < instances; i++){
        KDL::Vector curr(0.2 + y[i]/6., 0.1 + x[i]/6., 0.1);
        KDL::Vector n(curr.x(), curr.y(), 0); n.Normalize();
        KDL::Vector s(curr.y(), -curr.x(), 0); s.Normalize();
        KDL::Vector a(0, 0, -1);
        KDL::Rotation R(n, s, a);

        Eigen::VectorXf goal { prev_q->getCoord() };
        goal(0) = std::atan2(curr.y(), curr.x());
        if (i != 0)
            q = Robot::getRobot()->computeInverseKinematics(R, curr, std::make_shared<base::RealVectorSpaceState>(goal));
        else
            q = Robot::getRobot()->computeInverseKinematics(R, curr);
        if (q == nullptr || (i != 0 && std::abs(prev_q->getCoord(3) - q->getCoord(3)) > 0.1)){
            k++;
            if (k == 500) continue;
            i--;
            continue;
        }
        k = 0;
     
        std::shared_ptr<base::State> q_copy = q;
        if (std::abs(prev_q->getCoord(0) - q->getCoord(0)) > M_PI)
        {
            if (q->getCoord(0) > 0)
                q->setCoord(q->getCoord(0) - 2*M_PI, 0);
            else
                q->setCoord(q->getCoord(0) + 2*M_PI, 0);
        }
        path.emplace_back(q);
        time_instances.emplace_back(T/instances * (i + 1) + 3);
        prev_q = q_copy;
    }

    Trajectory::clear();
    Trajectory::addPath(path, time_instances);
    Trajectory::publish();
    return true;
}


