#include "sim_demos/RealTimePlanningNode.h"

typedef planning::drbt::DRGBT DP;    // 'DP' is Dynamic Planner

sim_bringup::RealTimePlanningNode::RealTimePlanningNode(const std::string &node_name, const std::string &config_file_path, bool loop_, 
                                                        const std::string &output_file_name) : 
    BaseNode(node_name, config_file_path),
    AABB(config_file_path),
    DP(Planner::scenario->getStateSpace(), Planner::scenario->getStart(), Planner::scenario->getGoal())
{
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };

    AABB::setEnvironment(Planner::scenario->getEnvironment());
    if (AABB::getMinNumCaptures() == 1)
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::callback, this, std::placeholders::_1));
    else
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::withFilteringCallback, this, std::placeholders::_1));

    YAML::Node real_time_node { node["real_time"] };
    std::string real_time_scheduling { real_time_node["scheduling"].as<std::string>() };
    if (real_time_scheduling == "FPS")
        DRGBTConfig::REAL_TIME_SCHEDULING = planning::RealTimeScheduling::FPS;
    else if (real_time_scheduling == "None")
        DRGBTConfig::REAL_TIME_SCHEDULING = planning::RealTimeScheduling::None;

    DRGBTConfig::MAX_PLANNING_TIME = INFINITY;
    DRGBTConfig::MAX_TIME_TASK1 = real_time_node["max_time_task1"].as<float>();
    DRGBTConfig::MAX_ITER_TIME = BaseNode::period;
    DRGBTConfig::STATIC_PLANNER_TYPE = Planner::getPlannerType();
    if (DRGBTConfig::STATIC_PLANNER_TYPE == planning::PlannerType::RGBMTStar)
        RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = true;
    
    replanning_result = -1;
    loop = loop_;
    q_start_init = Planner::scenario->getStart();
    q_goal_init = Planner::scenario->getGoal();
    max_error = Eigen::VectorXf::Zero(Robot::getNumDOFs());

    if (!output_file_name.empty())
    {
        std::cout << "Recorded data will be saved to: " 
                  << project_abs_path + config_file_path.substr(0, config_file_path.size()-5) + output_file_name << "\n";
        output_file.open(project_abs_path + config_file_path.substr(0, config_file_path.size()-5) + output_file_name, std::ofstream::out);
        recording_trajectory_timer = this->create_wall_timer(std::chrono::microseconds(size_t(Trajectory::getTrajectoryMaxTimeStep() * 1e6)), 
                                     std::bind(&RealTimePlanningNode::recordingTrajectoryCallback, this));
    }
}

void sim_bringup::RealTimePlanningNode::planningCallback()
{
    if (DP::q_start == DP::q_goal)
        return;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------------------");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Iteration num. %ld", DP::planner_info->getNumIterations());
    DP::time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock
    AABB::updateEnvironment();

    if (replanning_result == 1)  // New path is found within the specified time limit, thus update predefined path to the goal
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The path has been replanned in %f [ms].", Planner::getPlanningTime() * 1e3);
        Planner::preprocessPath(Planner::getPath(), DP::predefined_path, DP::max_edge_length);
        DP::horizon.clear();
        DP::status = base::State::Status::Reached;
        DP::replanning = false;
        DP::q_next = std::make_shared<planning::drbt::HorizonState>(DP::q_current, 0, DP::q_current);
        replanning_result = -1;
    }
    else if (replanning_result == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Replanning is required. New path is not found! ");
        DP::replanning = true;
    }

    switch (DP::planner_info->getNumIterations())
    {
    case 0:
        if (!Robot::isReady())
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Waiting to set up the robot...");
            return;
        }
        if (!AABB::isReady())
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Waiting to set up the environment...");
            return;
        }

        // ------------------------------------------------------------------------------- //
        // Initial iteration: Obtaining an inital path using specified static planner
        DP::time_alg_start = DP::time_iter_start;       // Start the algorithm clock
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obtaining an inital path...");
        replan(DRGBTConfig::MAX_ITER_TIME - 2e-3);      // 2 [ms] is reserved for other lines
        
        break;
    
    default:
        // ------------------------------------------------------------------------------- //
        // Current robot position and velocity (measured vs computed)
        DP::q_current = DP::ss->getNewState(DP::splines->spline_next->getPosition(DP::splines->spline_next->getTimeCurrent(true)));
        // std::cout << "Current position (measured): " << Robot::getJointsPositionPtr() << "\n";
        // std::cout << "Current position (computed): " << DP::q_current << "\n";
        // std::cout << "Current velocity (measured): " << Robot::getJointsVelocityPtr() << "\n";
        // std::cout << "Current velocity (computed): " << DP::ss->getNewState(DP::splines->spline_next->getVelocity(DP::splines->spline_next->getTimeCurrent(true))) << "\n";
        
        // ------------------------------------------------------------------------------- //
        // Checking whether the collision occurs
        if (!DP::ss->isValid(DP::q_current))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "********** Robot is stopping. Collision has been occurred!!! **********");
            DP::horizon.clear();
            DP::status = base::State::Status::Trapped;
            DP::replanning = true;
            DP::q_next = std::make_shared<planning::drbt::HorizonState>(DP::q_current, -1, DP::q_current);

            // If you want to terminate the algorithm, uncomment the following:
            // DP::planner_info->setSuccessState(false);
            // DP::planner_info->setPlanningTime(DP::planner_info->getIterationTimes().back());
            // rclcpp::shutdown();
            
            return;     // The algorithm will still continue its execution.
        }
        
        // ------------------------------------------------------------------------------- //
        taskComputingNextConfiguration();
        taskReplanning();

        break;
    }

    // ------------------------------------------------------------------------------- //
    // Checking the real-time execution
    float time_iter_remain = DRGBTConfig::MAX_ITER_TIME * 1e3 - DP::getElapsedTime(DP::time_iter_start, planning::TimeUnit::ms);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Remaining iteration time is %f [ms].", time_iter_remain);
    if (time_iter_remain < 0)
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "********** Real-time is broken. %f [ms] exceeded!!! **********", -time_iter_remain);

    // ------------------------------------------------------------------------------- //
    // Planner info and terminating condition
    DP::planner_info->setNumIterations(DP::planner_info->getNumIterations() + 1);
    DP::planner_info->addIterationTime(DP::getElapsedTime(DP::time_alg_start));
    if (DP::checkTerminatingCondition(DP::status))
    {
        if (loop)
        {
            DP::q_start = q_goal_init;
            DP::q_goal = q_start_init;
            q_start_init = DP::q_start;
            q_goal_init = DP::q_goal;
            DP::planner_info->setNumIterations(0);  // Go from the beginning
        }
        else
            rclcpp::shutdown();
    }
    
}

void sim_bringup::RealTimePlanningNode::taskComputingNextConfiguration()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TASK 1: Computing next configuration... ");
    
    // Since the environment may change, a new distance is required!
    DP::ss->computeDistance(DP::q_current, true);
    
    if (DP::status != base::State::Status::Advanced)
        DP::generateHorizon();

    DP::updateHorizon();
    DP::generateGBur();
    DP::computeNextState();
    computeTrajectory();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Elapsed time for TASK 1: %f [ms].", DP::getElapsedTime(DP::time_iter_start, planning::TimeUnit::ms));
}

void sim_bringup::RealTimePlanningNode::taskReplanning()
{
    if (DP::whetherToReplan())
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TASK 2: Replanning... ");
        if (Planner::isReady())
            replan(DRGBTConfig::MAX_ITER_TIME - DP::getElapsedTime(DP::time_iter_start) - 2e-3);    // 2 [ms] is reserved for other lines
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Planner is not ready! ");
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning is not required! ");
}

/// @brief Try to replan the predefined path from 'q_current' to 'q_goal' during a specified time limit 'max_planning_time'.
/// @param max_planning_time Maximal (re)planning time in [s].
void sim_bringup::RealTimePlanningNode::replan(float max_planning_time)
{
    replanning_result = false;
    try
    {
        if (max_planning_time < 0)
            throw std::runtime_error("Not enough time for replanning! ");

        switch (DRGBTConfig::REAL_TIME_SCHEDULING)
        {
        case planning::RealTimeScheduling::FPS:
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning with Fixed Priority Scheduling ");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to replan in %f [ms]...", max_planning_time * 1e3);
            std::thread replanning_thread([this, &max_planning_time]() 
            {
                replanning_result = Planner::solve(DP::q_current, DP::q_goal, max_planning_time);
            });
            replanning_thread.detach();
            break;
        }
        case planning::RealTimeScheduling::None:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning without real-time scheduling ");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to replan in %f [ms]...", max_planning_time * 1e3);
            replanning_result = Planner::solve(DP::q_current, DP::q_goal, max_planning_time);
            break;
        }
        
    }
    catch (std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", e.what());
    }
}

/// @brief Compute trajectory points from 'spline_next' and publish them.
void sim_bringup::RealTimePlanningNode::computeTrajectory()
{
    float t_delay { DP::updateCurrentState(true) };
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New trajectory is computed! Delay time: %f [ms]", t_delay * 1e3);
    if (DP::splines->spline_next != DP::splines->spline_current)  // New spline is computed
        DP::splines->spline_next->setTimeStart(t_delay);

    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    Trajectory::clear();
    Trajectory::addPoints(DP::splines->spline_next, 
                          DP::splines->spline_next->getTimeCurrent(), 
                          DP::splines->spline_next->getTimeEnd() + DRGBTConfig::MAX_TIME_TASK1);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Elapsed time: %f [ms] for adding %ld points", 
    //             DP::getElapsedTime(time_start_) * 1e3, Trajectory::getNumPoints());

    while (DP::getElapsedTime(time_start_) < t_delay) {}    // Wait for 't_delay' to exceed...
    Trajectory::publish();
}

void sim_bringup::RealTimePlanningNode::recordingTrajectoryCallback()
{
    if (DP::getPlannerInfo()->getNumIterations() == 0)
        return;

    float time_spline { DP::splines->spline_next->getTimeCurrent(true) };
    output_file << "Time [s]: \n";
    output_file << DP::getElapsedTime(DP::time_alg_start) << "\n";

    output_file << "Position (referent): \n";
    Eigen::VectorXf pos_ref { DP::splines->spline_next->getPosition(time_spline) };
    output_file << pos_ref.transpose() << "\n";
    output_file << "Position (measured): \n";
    output_file << Robot::getJointsPosition().transpose() << "\n";

    output_file << "Velocity (referent): \n";
    output_file << DP::splines->spline_next->getVelocity(time_spline).transpose() << "\n";
    output_file << "Velocity (measured): \n";
    output_file << Robot::getJointsVelocity().transpose() << "\n";

    // output_file << "Acceleration (referent): \n";
    // output_file << DP::splines->spline_next->getAcceleration(time_spline).transpose() << "\n";
    // output_file << "Acceleration (measured): \n";
    // output_file << Robot::getJointsAcceleration().transpose() << "\n";

    output_file << "--------------------------------------------------------------------\n";

    Eigen::VectorXf error { (pos_ref - Robot::getJointsPosition()).cwiseAbs() };
    max_error = max_error.cwiseMax(error);
    std::cout << "Curr. error: " << error.transpose() << "\n";
    std::cout << "Max. error:  " << max_error.transpose() << "\n";
}
