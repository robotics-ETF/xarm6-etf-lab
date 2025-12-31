#include "base/Trajectory.h"

sim_bringup::Trajectory::Trajectory(const std::string &config_file_path) : 
    Robot(config_file_path)
{
    try
    {
        if (Robot::getNumDOFs() == 6)
            msg.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        else
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Such number of robot DOFs is not supported!");

        std::string project_abs_path(__FILE__);
        for (size_t i = 0; i < 4; i++)
            project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
        
        YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
        YAML::Node planner_node { node["planner"] };
        if (planner_node["trajectory_max_time_step"].IsDefined())
            trajectory_max_time_step = planner_node["trajectory_max_time_step"].as<float>();
        else
        {
            trajectory_max_time_step = 0.1;
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Maximal edge length is not defined! Using default value of %f", trajectory_max_time_step);
        }

        if (planner_node["trajectory_recording"].IsDefined())
        {
            trajectory_recording = planner_node["trajectory_recording"].as<bool>();
            if (trajectory_recording)
            {
                std::string filename { project_abs_path + config_file_path.substr(0, config_file_path.size()-5) + "_traj_recording.log" };
                std::cout << "Recorded trajectory will be saved to: " << filename << "\n";
                output_file.open(filename, std::ofstream::out);
                time_recording = std::chrono::steady_clock::now();
            }
        }
        else
            trajectory_recording = false;

        ready = false;
        
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << "\n";
    }
}

void sim_bringup::Trajectory::addPoint(float time_instance, const Eigen::VectorXf &position)
{
    trajectory_msgs::msg::JointTrajectoryPoint point {};
    for (long int i = 0; i < position.size(); i++)
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
    trajectory_msgs::msg::JointTrajectoryPoint point {};
    for (long int i = 0; i < position.size(); i++)
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
    trajectory_msgs::msg::JointTrajectoryPoint point {};
    for (long int i = 0; i < position.size(); i++)
    {
        point.positions.emplace_back(position(i));
        point.velocities.emplace_back(velocity(i));
        point.accelerations.emplace_back(acceleration(i));
    }

    point.time_from_start.sec = int32_t(time_instance);
    point.time_from_start.nanosec = (time_instance - point.time_from_start.sec) * 1e9;
    msg.points.emplace_back(point);
}

/// @brief Add points from 'trajectory' to 'msg.points' using a time discretization step 'trajectory_max_time_step'.
/// @param trajectory Trajectory whose points are added.
/// @param t_begin Time instance from 'trajectory' which determines a first point that will be added.
/// @param t_end Time instance from 'trajectory' which determines a last point that will be added.
/// @param t_offset Time offset for which all points are time shifted. Default: 0.
void sim_bringup::Trajectory::addPoints(const std::shared_ptr<planning::trajectory::AbstractTrajectory> trajectory, 
                                        float t_begin, float t_end, float t_offset)
{
    float t { t_begin };
    
    do
    {
        t += trajectory_max_time_step;
        if (t > t_end)
            t = t_end;

        addPoint
        (
            t - t_begin + t_offset, 
            trajectory->getPosition(t), 
            trajectory->getVelocity(t), 
            trajectory->getAcceleration(t)
        );
        
        // std::cout << "Adding point at time: " << t - t_begin + t_offset << " [s] \t Trajectory time: " << t << " [s] \n";
        // std::cout << "Position:     " << trajectory->getPosition(t).transpose() << "\n";
        // std::cout << "Velocity:     " << trajectory->getVelocity(t).transpose() << "\n";
        // std::cout << "Acceleration: " << trajectory->getAcceleration(t).transpose() << "\n\n";
    } 
    while (t < t_end);
}

void sim_bringup::Trajectory::addTrajectory(const std::vector<std::shared_ptr<base::State>> &path, const std::vector<float> &time_instances)
{
    for (size_t i = 0; i < path.size(); i++)
        addPoint(time_instances[i], path[i]->getCoord());
}

void sim_bringup::Trajectory::addTrajectory(const std::vector<Eigen::VectorXf> &path, const std::vector<float> &time_instances)
{
    for (size_t i = 0; i < path.size(); i++)
        addPoint(time_instances[i], path[i]);
}

void sim_bringup::Trajectory::addTrajectory(const std::shared_ptr<planning::trajectory::AbstractTrajectory> trajectory)
{
    if (trajectory == nullptr)
        return;
    
    for (float t = 0; t <= trajectory->getTimeFinal(); t += trajectory_max_time_step)
    {
        addPoint
        (
            t, 
            trajectory->getPosition(t), 
            trajectory->getVelocity(t), 
            trajectory->getAcceleration(t)
        );
        
        // std::cout << "Adding point at time: " << t << " [s] \t Trajectory time: " << t << " [s] \n";
        // std::cout << "Position:     " << trajectory->getPosition(t).transpose() << "\n";
        // std::cout << "Velocity:     " << trajectory->getVelocity(t).transpose() << "\n";
        // std::cout << "Acceleration: " << trajectory->getAcceleration(t).transpose() << "\n\n";
    }
}

/// @brief Publish a trajectory stored in 'msg.points'.
/// @param print Whether to print a published trajectory points. Default: false.
void sim_bringup::Trajectory::publish(bool print)
{
    if (msg.points.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "There are no trajectory points to be published!");
        return;
    }

    publisher->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing trajectory ...");

    if (print)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory points: ");
        for (size_t i = 0; i < msg.points.size(); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Num. %ld.\t Time: %f [s]", 
                        i, (msg.points[i].time_from_start.sec + msg.points[i].time_from_start.nanosec * 1e-9));

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Position: ");
            for (size_t idx = 0; idx < msg.points[i].positions.size(); idx++)
                std::cout << msg.points[i].positions[idx] << "\t";
            std::cout << "\n";

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Velocity: ");
            for (size_t idx = 0; idx < msg.points[i].velocities.size(); idx++)
                std::cout << msg.points[i].velocities[idx] << "\t";
            std::cout << "\n";

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Acceleration: ");
            for (size_t idx = 0; idx < msg.points[i].accelerations.size(); idx++)
                std::cout << msg.points[i].accelerations[idx] << "\t";
            std::cout << "\n";
        }
    }
}

void sim_bringup::Trajectory::clear()
{
    msg.points.clear();
}
