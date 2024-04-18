#include "demos/RealTimePlanningNode.h"

typedef planning::drbt::DRGBT DP;    // 'DP' is Dynamic Planner

sim_bringup::RealTimePlanningNode::RealTimePlanningNode(const std::string &node_name, const std::string &config_file_path) : 
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
    
}

void sim_bringup::RealTimePlanningNode::planningCallback()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Iteration num. %ld", DP::planner_info->getNumIterations());
    DP::time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock
    AABB::updateEnvironment();

    switch (DP::planner_info->getNumIterations())
    {
    case 0:
        if (!Robot::isReady())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting to set up the robot...");
            return;
        }
        if (!AABB::isReady())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting to set up the environment...");
            return;
        }

        // ------------------------------------------------------------------------------- //
        // Initial iteration: Obtaining an inital path using specified static planner
        DP::time_alg_start = DP::time_iter_start;      // Start the algorithm clock
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obtaining an inital path...");
        replan(DRGBTConfig::MAX_ITER_TIME);
        
        break;
    
    default:        
        // ------------------------------------------------------------------------------- //
        // Current robot position and velocity (measured vs computed)
        DP::q_current = DP::ss->getNewState(DP::spline_next->getPosition(DP::spline_next->getTimeCurrent(true)));
        // std::cout << "q_current (measured): " << Robot::getJointsPositionPtr() << "\n";
        // std::cout << "q_current (computed): " << DP::q_current << "\n";
        // std::cout << "q_current_dot (measured): " << Robot::getJointsVelocityPtr() << "\n";
        // std::cout << "q_current_dot (computed): " << DP::ss->getNewState(DP::spline_next->getVelocity(DP::spline_next->getTimeCurrent(true))) << "\n";
        
        // ------------------------------------------------------------------------------- //
        // Checking whether the collision occurs
        if (!DP::ss->isValid(DP::q_current))
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "********** Robot is stopping. Collision has been occurred!!! **********");
            DP::q_target = DP::q_current;
            DP::clearHorizon(base::State::Status::Trapped, true);
            DP::q_next = std::make_shared<planning::drbt::HorizonState>(DP::q_target, 0);
            DP::q_next->setStateReached(DP::q_target);

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
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "********** Real-time is broken. %f [ms] exceeded!!! **********", -time_iter_remain);

    // ------------------------------------------------------------------------------- //
    // Planner info and terminating condition
    DP::planner_info->setNumIterations(DP::planner_info->getNumIterations() + 1);
    DP::planner_info->addIterationTime(DP::getElapsedTime(DP::time_alg_start));
    if (DP::checkTerminatingCondition(DP::status))
        rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------------------\n");
}

void sim_bringup::RealTimePlanningNode::taskComputingNextConfiguration()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TASK 1: Computing next configuration... ");
    
    // Since the environment may change, a new distance is required!
    float d_c { DP::ss->computeDistance(DP::q_target, true) };
    if (d_c <= 0)   // The desired/target conf. is not safe, thus the robot is required to stop immediately, 
    {               // and compute the horizon again from 'q_current'
        // TODO: Emergency stopping needs to be implemented using quartic spline.
        DP::q_target = DP::q_current;
        d_c = DP::ss->computeDistance(DP::q_target, true);
        DP::clearHorizon(base::State::Status::Trapped, true);
        DP::q_next = std::make_shared<planning::drbt::HorizonState>(DP::q_target, 0);
        DP::q_next->setStateReached(DP::q_target);
        // std::cout << "Not updating the robot current state since d_c < 0. \n";
    }
    
    if (DP::status != base::State::Status::Advanced)
        DP::generateHorizon();

    DP::updateHorizon(d_c);
    DP::generateGBur();
    DP::computeNextState();
    computeTrajectory();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Elapsed time: %f [ms].", DP::getElapsedTime(DP::time_iter_start, planning::TimeUnit::ms));
}

void sim_bringup::RealTimePlanningNode::taskReplanning()
{
    if (DP::whetherToReplan())
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TASK 2: Replanning... ");
        if (Planner::isReady()) 
        {
            replan(DRGBTConfig::MAX_ITER_TIME - DP::getElapsedTime(DP::time_iter_start) - 2e-3);    // 2 [ms] is subtracted because of the following code lines
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Elapsed time: %f [ms].", DP::getElapsedTime(DP::time_iter_start, planning::TimeUnit::ms));
        }
        else
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner is not ready! ");
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning is not required! ");
}

// Try to replan the predefined path from the target to the goal configuration within the specified time
void sim_bringup::RealTimePlanningNode::replan(float max_planning_time)
{
    bool result { false };
    std::thread replanning_thread {};

    try
    {
        if (max_planning_time < 0)
            throw std::runtime_error("Not enough time for replanning! ");

        switch (DRGBTConfig::REAL_TIME_SCHEDULING)
        {
        case planning::RealTimeScheduling::FPS:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning with Fixed Priority Scheduling ");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to replan in %f [ms]...", max_planning_time * 1e3);
            replanning_thread = std::thread([this, &result, &max_planning_time]() 
            {
                result = Planner::solve(DP::q_target, DP::q_goal, max_planning_time);
            });
            std::this_thread::sleep_for(std::chrono::microseconds(size_t(max_planning_time * 1e6)));
            break;
        
        case planning::RealTimeScheduling::None:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning without real-time scheduling ");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to replan in %f [s]...", max_planning_time);
            result = Planner::solve(DP::q_target, DP::q_goal, max_planning_time);
            break;
        }

        // New path is found within the specified time limit, thus update predefined path to the goal
        if (result)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The path has been replanned in %f [ms].", Planner::getPlanningTime() * 1e3);
            Planner::preprocessPath(Planner::getPath(), DP::predefined_path, DP::max_edge_length);
            DP::clearHorizon(base::State::Status::Reached, false);
            DP::q_next = std::make_shared<planning::drbt::HorizonState>(DP::q_target, 0);
        }

        replanning_thread.detach();

        if (!result)    // New path is not found
            throw std::runtime_error("New path is not found! ");
    }
    catch (std::exception &e)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning is required. %s", e.what());
        DP::replanning = true;
    }
}

/// @brief Compute trajectory points from 'spline_next' and publish them.
void sim_bringup::RealTimePlanningNode::computeTrajectory()
{
    float t_delay { DP::updateCurrentState(true) };

    if (DP::spline_next == DP::spline_current)  // Trajectory has been already computed!
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Not computing a new trajectory! ");
        return;
    }

    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    Trajectory::clear();
    Trajectory::addPoints(DP::spline_next, t_delay, DP::spline_next->getTimeFinal());
    Trajectory::publish();
    float t_publish { DP::getElapsedTime(time_start_) };

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Delay time:   %f [ms]", t_delay * 1e3);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publish time: %f [ms]", t_publish * 1e3);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sleep time:   %f [ms]", (t_delay - t_publish) * 1e3);

    std::this_thread::sleep_for(std::chrono::microseconds(size_t((t_delay - t_publish) * 1e6)));
    DP::spline_next->setTimeStart();
}
