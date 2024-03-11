#include "base/Trajectory.h"

sim_bringup::Trajectory::Trajectory(const std::string &config_file_path)
{
    std::string project_abs_path = std::string(__FILE__);
    for (int i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node = YAML::LoadFile(project_abs_path + config_file_path);

    if (node["robot"]["num_DOFs"].as<int>() == 6)
        msg.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Such number of robot DOFs is not supported!");
}

void sim_bringup::Trajectory::addPoint(float time_instance, const Eigen::VectorXf &position)
{
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (int i = 0; i < position.size(); i++)
    {
        point.positions.emplace_back(position(i));
        point.velocities.emplace_back(0);
        point.accelerations.emplace_back(0);
    }

    point.time_from_start.sec = int32_t(time_instance);
    point.time_from_start.nanosec = (time_instance - point.time_from_start.sec) * 1e9;
    msg.points.emplace_back(point);
}

void sim_bringup::Trajectory::addPoint(float time_instance, const Eigen::VectorXf &position, const Eigen::VectorXf &velocity)
{
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (int i = 0; i < position.size(); i++)
    {
        point.positions.emplace_back(position(i));
        point.velocities.emplace_back(velocity(i));
        point.accelerations.emplace_back(0);
    }

    point.time_from_start.sec = int32_t(time_instance);
    point.time_from_start.nanosec = (time_instance - point.time_from_start.sec) * 1e9;
    msg.points.emplace_back(point);
}

void sim_bringup::Trajectory::addPoint(float time_instance, const Eigen::VectorXf &position, const Eigen::VectorXf &velocity, 
                                       const Eigen::VectorXf &acceleration)
{
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (int i = 0; i < position.size(); i++)
    {
        point.positions.emplace_back(position(i));
        point.velocities.emplace_back(velocity(i));
        point.accelerations.emplace_back(acceleration(i));
    }

    point.time_from_start.sec = int32_t(time_instance);
    point.time_from_start.nanosec = (time_instance - point.time_from_start.sec) * 1e9;
    msg.points.emplace_back(point);
}

void sim_bringup::Trajectory::addPath(const std::vector<std::shared_ptr<base::State>> &path, const std::vector<float> &time_instances)
{
    for (int i = 0; i < path.size(); i++)
        addPoint(time_instances[i], path[i]->getCoord());
}

void sim_bringup::Trajectory::addPath(const std::vector<Eigen::VectorXf> &path, const std::vector<float> &time_instances)
{
    for (int i = 0; i < path.size(); i++)
        addPoint(time_instances[i], path[i]);
}

void sim_bringup::Trajectory::addPath(const std::vector<std::shared_ptr<base::State>> &path)
{
    // TODO: Implementirati preko splinea
}

void sim_bringup::Trajectory::publish()
{
    if (msg.points.empty())
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "There is no trajectory to publish!");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory: ");
    for (int i = 0; i < msg.points.size(); i++)
    {
        if (msg.points[i].positions.size() == 6)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Num. %d.\t Time: %f [s].\t Point: (%f, %f, %f, %f, %f, %f)", 
                        i, msg.points[i].time_from_start.sec + msg.points[i].time_from_start.nanosec * 1e-9, 
                        msg.points[i].positions[0], 
                        msg.points[i].positions[1],
                        msg.points[i].positions[2],
                        msg.points[i].positions[3],
                        msg.points[i].positions[4],
                        msg.points[i].positions[5]);
        }
    }

    msg.header.stamp.sec = 0;
    msg.header.stamp.nanosec = 0;
    publisher->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing trajectory ...");
}

void sim_bringup::Trajectory::clear()
{
    msg.points.clear();
}