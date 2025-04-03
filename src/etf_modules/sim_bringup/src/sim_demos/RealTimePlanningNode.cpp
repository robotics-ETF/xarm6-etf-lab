#include "sim_demos/RealTimePlanningNode.h"

typedef planning::drbt::DRGBT DP;    // 'DP' is Dynamic Planner

sim_bringup::RealTimePlanningNode::RealTimePlanningNode(const std::string &node_name, const std::string &config_file_path, 
                                                        bool loop_execution_, const std::string &output_file_name) : 
    BaseNode(node_name, config_file_path),
    AABB(config_file_path),
    DP(Planner::scenario->getStateSpace(), Planner::scenario->getStart(), Planner::scenario->getGoal())
{
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };

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
    DRGBTConfig::GUARANTEED_SAFE_MOTION = node["planner"]["guaranteed_safe_motion"].as<bool>();
    trajectory_advance_time = node["planner"]["trajectory_advance_time"].as<float>();
    max_obs_vel = node["planner"]["max_obs_vel"].as<float>();
    
    iteration_completed = true;
    planning_result = -1;
    replanning_result = -1;
    loop_execution = loop_execution_;
    q_start_init = DP::q_start;
    q_goal_init = DP::q_goal;

    callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Enable all callbacks to run concurrently
    timer = this->create_wall_timer(std::chrono::microseconds(size_t(period * 1e6)), std::bind(&BaseNode::baseCallback, this), callback_group);
    replanning_service = this->create_service<std_srvs::srv::Empty>("replanning_service",
                         std::bind(&RealTimePlanningNode::replanningCallback, this, std::placeholders::_1, std::placeholders::_2), 
                         rmw_qos_profile_services_default, callback_group);
    replanning_client = this->create_client<std_srvs::srv::Empty>("replanning_service", rmw_qos_profile_services_default, callback_group);
    
    if (!output_file_name.empty())
    {
        std::cout << "Recorded data will be saved to: " 
                  << project_abs_path + config_file_path.substr(0, config_file_path.size()-5) + output_file_name << "\n";
        output_file.open(project_abs_path + config_file_path.substr(0, config_file_path.size()-5) + output_file_name, std::ofstream::out);
        recording_trajectory_timer = this->create_wall_timer(std::chrono::microseconds(size_t(Trajectory::getTrajectoryMaxTimeStep() * 1e6)), 
                                     std::bind(&RealTimePlanningNode::recordingTrajectoryCallback, this), callback_group);
        max_error = Eigen::VectorXf::Zero(Robot::getNumDOFs());
    }
}

void sim_bringup::RealTimePlanningNode::planningCallback()
{
    if (!iteration_completed)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "********** Real-time is broken!!! **********");
        return;
    }

    if (DP::q_start == DP::q_goal)  // There is nothing to plan!
        return;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------------------");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Iteration num. %ld", DP::planner_info->getNumIterations());
    DP::time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock
    iteration_completed = false;
    planning_result = -1;
    AABB::updateEnvironment(DP::ss->env, max_obs_vel);
    
    // ------------------------------------------------------------------------------- //
    // Current robot position and velocity (measured vs computed)
    DP::q_current = DP::ss->getNewState(DP::splines->spline_next->getPosition(DP::splines->spline_next->getTimeCurrent(true)));
    // DP::q_current = Robot::getJointsPositionPtr();
    // std::cout << "Current position (measured): " << Robot::getJointsPositionPtr() << "\n";
    // std::cout << "Current position (computed): " << DP::q_current << "\n";
    // std::cout << "Current velocity (measured): " << Robot::getJointsVelocityPtr() << "\n";
    // std::cout << "Current velocity (computed): " << DP::ss->getNewState(DP::splines->spline_next->getVelocity(DP::splines->spline_next->getTimeCurrent(true))) << "\n";
    // ------------------------------------------------------------------------------- //

    if (replanning_result == 1)  // New path is found within the specified time limit, thus update predefined path to the goal
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The path has been replanned in %f [ms].", Planner::getPlanningTime() * 1e3);
        Planner::preprocessPath(Planner::getPath(), DP::predefined_path, DP::max_edge_length);
        DP::horizon.clear();
        DP::status = base::State::Status::Reached;
        DP::replanning_required = false;
        DP::q_next = std::make_shared<planning::drbt::HorizonState>(DP::q_current, 0, DP::q_current);
        replanning_result = -1;
    }
    else if (replanning_result == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Replanning is required. New path is not found! ");
        DP::replanning_required = true;
    }

    switch (DP::planner_info->getNumIterations())
    {
    case 0:
        if (!Robot::isReady())
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Waiting to set up the robot...");
            iteration_completed = true;
            return;
        }
        if (!AABB::isReady())
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Waiting to set up the environment...");
            iteration_completed = true;
            return;
        }

        // ------------------------------------------------------------------------------- //
        // Initial iteration: Obtaining an inital path using specified static planner
        DP::time_alg_start = DP::time_iter_start;       // Start the algorithm clock
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obtaining an inital path...");
        taskReplanning(true);
        
        break;
    
    default:
        // ------------------------------------------------------------------------------- //
        // Checking whether the collision occurs
        if (!DP::ss->isValid(DP::q_current))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "********** Robot is stopping. Collision has been occurred!!! **********");
            DP::horizon.clear();
            DP::status = base::State::Status::Trapped;
            DP::replanning_required = true;
            DP::q_next = std::make_shared<planning::drbt::HorizonState>(DP::q_current, -1, DP::q_current);
            iteration_completed = true;
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
        if (loop_execution)
        {
            DP::q_start = q_goal_init;
            DP::q_goal = q_start_init;
            q_start_init = DP::q_start;
            q_goal_init = DP::q_goal;

            taskReplanning(true);
        }
        else
        {
            DP::q_start = DP::q_goal;   // Robot will automatically stop and wait until 'DP::q_goal' is changed
            planning_result = DP::planner_info->getSuccessState();
        }
    }
    
    iteration_completed = true;
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

void sim_bringup::RealTimePlanningNode::taskReplanning(bool replanning_required_explicitly)
{
    if (replanning_required_explicitly)
        DP::replanning_required = true;
    
    if (DP::whetherToReplan())
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TASK 2: Replanning... ");
        if (Planner::isReady())
        {
            std::shared_ptr<std_srvs::srv::Empty::Request> request { std::make_shared<std_srvs::srv::Empty::Request>() };
            replanning_client->async_send_request(request);
        }
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Planner is not ready! ");
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning is not required! ");
}

/// @brief Try to replan the predefined path from 'q_current' to 'q_goal' during a specified time limit 'max_replanning_time'.
void sim_bringup::RealTimePlanningNode::replanningCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                           [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    try
    {
        replanning_result = false;
        float max_replanning_time = DRGBTConfig::MAX_ITER_TIME - DP::getElapsedTime(DP::time_iter_start) - 1e-3;  // 1 [ms] is reserved for other lines
        
        if (max_replanning_time <= 0)
            throw std::runtime_error("Not enough time for replanning! ");

        switch (DRGBTConfig::REAL_TIME_SCHEDULING)
        {
        case planning::RealTimeScheduling::FPS:
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning with Fixed Priority Scheduling ");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to replan in %f [ms]...", max_replanning_time * 1e3);
            replanning_result = Planner::solve(DP::q_current, DP::q_goal, max_replanning_time);
            break;
        }
        case planning::RealTimeScheduling::None:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning without real-time scheduling ");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to replan in %f [ms]...", max_replanning_time * 1e3);
            replanning_result = Planner::solve(DP::q_current, DP::q_goal, max_replanning_time);
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

    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    Trajectory::clear();
    Trajectory::addPoints(DP::splines->spline_next, 
                          DP::splines->spline_next->getTimeCurrent() + trajectory_advance_time, 
                          DP::splines->spline_next->getTimeEnd() + DRGBTConfig::MAX_TIME_TASK1 + trajectory_advance_time);
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
    // std::cout << "Curr. error: " << error.transpose() << "\n";
    // std::cout << "Max. error:  " << max_error.transpose() << "\n";
}
