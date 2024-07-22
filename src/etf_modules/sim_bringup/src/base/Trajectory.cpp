#include "base/Trajectory.h"

sim_bringup::Trajectory::Trajectory(const std::string &config_file_path) : 
    Robot(config_file_path)
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

/// @brief Add points from 'spline' to 'msg.points' using a time discretization step 'trajectory_max_time_step'.
/// @param spline Spline whose points are added.
/// @param t_begin Time instance from 'spline' which determines a first point that will be added.
/// @param t_end Time instance from 'spline' which determines a last point that will be added.
/// @param t_offset Time offset for which all points are time shifted. Default: 0.
void sim_bringup::Trajectory::addPoints(std::shared_ptr<planning::trajectory::Spline> spline, float t_begin, float t_end, float t_offset)
{
    float t { t_begin };
    
    do
    {
        t += trajectory_max_time_step;
        if (t > t_end)
            t = t_end;

        addPoint(t - t_begin + t_offset, spline->getPosition(t), spline->getVelocity(t), spline->getAcceleration(t));
        
        // std::cout << "Adding point at time: " << t - t_begin + t_offset << " [s] \t Spline time: " << t << " [s] \n";
        // std::cout << "Position:     " << spline->getPosition(t).transpose() << "\n";
        // std::cout << "Velocity:     " << spline->getVelocity(t).transpose() << "\n";
        // std::cout << "Acceleration: " << spline->getAcceleration(t).transpose() << "\n\n";
    } 
    while (t < t_end);
}

void sim_bringup::Trajectory::addPath(const std::vector<std::shared_ptr<base::State>> &path, const std::vector<float> &time_instances)
{
    for (size_t i = 0; i < path.size(); i++)
        addPoint(time_instances[i], path[i]->getCoord());
}

void sim_bringup::Trajectory::addPath(const std::vector<Eigen::VectorXf> &path, const std::vector<float> &time_instances)
{
    for (size_t i = 0; i < path.size(); i++)
        addPoint(time_instances[i], path[i]);
}

/// @brief First method to add a path 'path' containing all points that robot should visit (not guaranteed). 
/// Converting this path to trajectory (i.e., assigning time instances to these points) will be automatically done by this function.
/// This is done by creating a sequence of quintic splines in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @param path Path containing all points that robot should visit.
/// @note Be careful since the distance between each two adjacent points from 'path' should not be too long! 
/// The robot motion between them is generally not a straight line in C-space. 
/// Consider using 'preprocessPath' function from 'Planner' class before using this function.
void sim_bringup::Trajectory::addPath(const std::vector<std::shared_ptr<base::State>> &path)
{
    std::shared_ptr<planning::trajectory::Spline> spline_current { nullptr };
    std::shared_ptr<planning::trajectory::Spline> spline_next { nullptr };

    addPoint(0, path.front()->getCoord());
    spline_current = std::make_shared<planning::trajectory::Spline5>
    (
        Robot::getRobot(), 
        path.front()->getCoord(), 
        Eigen::VectorXf::Zero(Robot::getNumDOFs()), 
        Eigen::VectorXf::Zero(Robot::getNumDOFs())
    );

    if (path.size() == 2)
        spline_current->compute(path[1]->getCoord());
    else
        spline_current->compute(path[2]->getCoord());
    
    float t_current { 0 };
    float t {}, t_min {}, t_max {};
    bool found { false };
    size_t num { 0 };
    const size_t max_num_iter { 5 };
    
    for (size_t i = 3; i < path.size(); i++)
    {
        // std::cout << "i: " << i << " ---------------------------\n";
        found = false;
        num = 0;
        t_min = 0;
        t_max = spline_current->getTimeFinal();

        while (true)
        {
            t = (t_min + t_max) / 2;
            // std::cout << "t: " << t << " [s] \n";

            spline_next = std::make_shared<planning::trajectory::Spline5>
            (
                Robot::getRobot(), 
                spline_current->getPosition(t), 
                spline_current->getVelocity(t), 
                spline_current->getAcceleration(t)
            );
            found = spline_next->compute(path[i]->getCoord());

            if (found)
                break;
            else if (++num == max_num_iter)
                t_min = t_max;  // Solution surely exists, and 'found' will become true.
            else
                t_min = t;
        }

        addPoints(spline_current, 0, t, t_current);        
        t_current += t;
        spline_current = spline_next;
        // std::cout << "t_current: " << t_current << " [s] \n";
    }

    addPoints(spline_current, 0, spline_current->getTimeFinal(), t_current);
}

/// @brief Second method to add a path 'path' containing all points that robot (must) visit. 
/// Converting this path to trajectory (i.e., assigning time instances to these points) will be automatically done by this function.
/// This is done by creating a sequence of quintic splines in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @param path Path containing all points that robot (must) visit.
/// @param must_visit Whether path points must be visited.
/// @note Be careful since the distance between each two adjacent points from 'path' should not be too long! 
/// The robot motion between them is generally not a straight line in C-space. 
/// Consider using 'preprocessPath' function from 'Planner' class before using this function.
void sim_bringup::Trajectory::addPath(const std::vector<std::shared_ptr<base::State>> &path, bool must_visit)
{
    std::vector<std::shared_ptr<planning::trajectory::Spline>> splines(path.size(), nullptr);
    bool found { false };
    size_t num_iter {};
    size_t max_num_iter { 5 };
    float delta_t_max {};
    Eigen::VectorXf q_final_dot_max {};
    Eigen::VectorXf q_final_dot_min {};
    Eigen::VectorXf q_final_dot {};
    Eigen::VectorXf q_final {};
    std::vector<float> vel_coeff(path.size(), 1.0);
    const float vel_coeff_const { 0.9 };
    auto time_start = std::chrono::steady_clock::now();
    float max_time { 1.0 };

    splines.front() = std::make_shared<planning::trajectory::Spline5>
    (
        Robot::getRobot(), 
        path.front()->getCoord(), 
        Eigen::VectorXf::Zero(Robot::getNumDOFs()), 
        Eigen::VectorXf::Zero(Robot::getNumDOFs())
    );
    
    for (size_t i = 1; i < path.size(); i++)
    {
        splines[i] = std::make_shared<planning::trajectory::Spline5>
        (
            Robot::getRobot(), 
            splines[i-1]->getPosition(splines[i-1]->getTimeFinal()), 
            splines[i-1]->getVelocity(splines[i-1]->getTimeFinal()), 
            splines[i-1]->getAcceleration(splines[i-1]->getTimeFinal())
        );

        if (i == path.size() - 1)   // Final configuration will be reached, thus final velocity and acceleration must be zero!
        {
            found = splines[i]->compute(path.back()->getCoord());
            if (!found) 
            {
                // std::cout << "Not found! \n";
                vel_coeff[--i] *= vel_coeff_const;
                --i;
            }
            else
                break;
        }

        if (!must_visit)
            q_final = (path[i-1]->getCoord() + path[i]->getCoord()) / 2;
        else
            q_final = path[i]->getCoord();
        
        found = false;
        num_iter = 0;
        delta_t_max = ((q_final - path[i-1]->getCoord()).cwiseQuotient(Robot::getMaxVel())).cwiseAbs().maxCoeff();
        q_final_dot_max = (q_final - path[i-1]->getCoord()) / delta_t_max;
        q_final_dot_min = Eigen::VectorXf::Zero(Robot::getNumDOFs());

        do
        {
            q_final_dot = vel_coeff[i] * (q_final_dot_max + q_final_dot_min) / 2;
            std::shared_ptr<planning::trajectory::Spline> spline_new {
                std::make_shared<planning::trajectory::Spline5>
                (
                    Robot::getRobot(), 
                    splines[i-1]->getPosition(splines[i-1]->getTimeFinal()), 
                    splines[i-1]->getVelocity(splines[i-1]->getTimeFinal()), 
                    splines[i-1]->getAcceleration(splines[i-1]->getTimeFinal())
                )
            };
            
            if (spline_new->compute(q_final, q_final_dot)) 
            {
                *splines[i] = *spline_new;
                q_final_dot_min = q_final_dot;
                found = true;
            }
            else
                q_final_dot_max = q_final_dot;
        }
        while (++num_iter < max_num_iter);

        if (!found)
        {
            found = splines[i]->compute(q_final);
            if (!found)
            {
                // std::cout << "Not found! \n";
                vel_coeff[--i] *= vel_coeff_const;
                --i;
            }
            // else std::cout << "Found with zero final velocity! \n";
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count() * 1e-3 > max_time)
        {
            found = false;
            break;
        }
    }

    if (found)
    {
        float t_current { 0 };
        addPoint(t_current, path.front()->getCoord());
        for (size_t i = 1; i < path.size(); i++)
        {
            addPoints(splines[i], 0, splines[i]->getTimeFinal(), t_current);
            t_current += splines[i]->getTimeFinal();
            // std::cout << "t_current: " << t_current << " [s] \n";
        }
    }
    else
        addPath(path);      // Add path using another method.
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

    if (!print)
        return;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory points: ");
    switch (msg.points.front().positions.size())
    {
    case 6:
        for (size_t i = 0; i < msg.points.size(); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Num. %ld.\t Time: %f [s]", 
                        i, (msg.points[i].time_from_start.sec + msg.points[i].time_from_start.nanosec * 1e-9));
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Position:     (%f, %f, %f, %f, %f, %f)", 
                        msg.points[i].positions[0],
                        msg.points[i].positions[1],
                        msg.points[i].positions[2],
                        msg.points[i].positions[3],
                        msg.points[i].positions[4],
                        msg.points[i].positions[5]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Velocity:     (%f, %f, %f, %f, %f, %f)", 
                        msg.points[i].velocities[0],
                        msg.points[i].velocities[1],
                        msg.points[i].velocities[2],
                        msg.points[i].velocities[3],
                        msg.points[i].velocities[4],
                        msg.points[i].velocities[5]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Acceleration: (%f, %f, %f, %f, %f, %f)", 
                        msg.points[i].accelerations[0],
                        msg.points[i].accelerations[1],
                        msg.points[i].accelerations[2],
                        msg.points[i].accelerations[3],
                        msg.points[i].accelerations[4],
                        msg.points[i].accelerations[5]);
        }
        break;
    
    default:
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot print trajectory!");
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing trajectory ...");
}

void sim_bringup::Trajectory::clear()
{
    msg.points.clear();
}
