#include "sim_demos/RealTimePlanningNode2.h"

typedef planning::rrtx::RRTx DP;    // 'DP' is Dynamic Planner

sim_bringup::RealTimePlanningNode2::RealTimePlanningNode2(const std::string &node_name, const std::string &config_file_path, 
                                                          bool loop_execution_) : 
    BaseNode(node_name, config_file_path),
    AABB(config_file_path),
    RRTx(Planner::scenario->getStateSpace(), Planner::scenario->getStart(), Planner::scenario->getGoal())
{
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };

    if (AABB::getMinNumCaptures() == 1)
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::callback, this, std::placeholders::_1));
    else
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::withFilteringCallback, this, std::placeholders::_1));

    RRTxConfig::MAX_PLANNING_TIME = INFINITY;
    RRTxConfig::MAX_ITER_TIME = BaseNode::period;
    trajectory_advance_time = node["planner"]["trajectory_advance_time"].as<float>();
    max_obs_vel = node["planner"]["max_obs_vel"].as<float>();
    
    iteration_completed = true;
    planning_result = -1;
    loop_execution = loop_execution_;
    q_start_init = DP::q_start;
    q_goal_init = DP::q_goal;
}

void sim_bringup::RealTimePlanningNode2::planningCallback()
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
        // Phase 1: Find an initial path (similar to RRT)
        DP::time_alg_start = std::chrono::steady_clock::now();
        DP::time_iter_start = DP::time_alg_start;
        first_path_found = false;
        while (!first_path_found)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obtaining an inital path... Tree size: %ld", DP::tree->getNumStates());
            q_rand = DP::ss->getRandomState();
            
            if (DP::generateRandomNumber(0, 1) < RRTxConfig::START_BIAS) {
                q_rand = DP::start_state;
            }
            
            q_near = DP::tree->getNearestState(q_rand);
            
            std::tie(status, q_new) = DP::extend(q_near, q_rand);
            
            if (status != base::State::Status::Trapped)
            {
                q_new->setCost(q_near->getCost() + DP::distance(q_near, q_new));
                DP::tree->upgradeTree(q_new, q_near);
                
                DP::rewireNeighbors(q_new);
                
                if (DP::distance(q_new, DP::start_state) < 2 * RRTxConfig::EPS_STEP && 
                    DP::ss->isValid(q_new, DP::start_state) && 
                    !DP::ss->robot->checkSelfCollision(q_new, DP::start_state))
                {
                    DP::start_state->setParent(q_new);
                    DP::start_state->setCost(q_new->getCost() + DP::distance(q_new, DP::start_state));
                    DP::tree->upgradeTree(DP::start_state, q_new);
                    DP::computePath();
                    first_path_found = true;
                }
            }
        }
        break;
    
    default:
        // Phase 2: Continue improving the solution
        // Start the iteration clock
        DP::time_iter_start = std::chrono::steady_clock::now();

        // Determine the shrinking ball radius
        if (RRTxConfig::R_REWIRE > 0)
            DP::r_rewire = RRTxConfig::R_REWIRE;
        else
            DP::r_rewire = DP::shrinkingBallRadius(DP::tree->getNumStates());

        // Sample a random state
        q_rand = DP::ss->getRandomState();
        
        // Find nearest neighbor
        q_near = DP::tree->getNearestState(q_rand);
        
        // DP::extend towards the random state
        std::tie(status, q_new) = DP::extend(q_near, q_rand);
        
        if (status != base::State::Status::Trapped)
        {
            // Compute initial cost
            q_new->setCost(q_near->getCost() + DP::distance(q_near, q_new));
            
            // Find neighbors within DP::r_rewire
            std::vector<std::shared_ptr<base::State>> neighbors = DP::findNeighbors(q_new, DP::r_rewire);
            
            // Choose parent that minimizes cost
            DP::chooseParent(q_new, neighbors);
            
            // Add to tree
            DP::tree->upgradeTree(q_new, q_new->getParent());
            
            // Rewire the tree
            DP::rewireNeighbors(q_new, neighbors);
            
            // Update the path if needed
            if (DP::updatePath()) {
                DP::computePath();
            }
        }

        computeTrajectory();

        // Change start to the current state
        DP::start_state = DP::q_current;

        // Update the path if needed
        if (DP::updatePath()) {
            DP::computePath();
        }
        // std::cout << "q_current_new: " << DP::q_current << "\n\n";

        // Checking the real-time execution
        // float time_iter_remain = RRTxConfig::MAX_ITER_TIME * 1e3 - DP::getElapsedTime(DP::time_iter_start, planning::TimeUnit::ms);
        // std::cout << "Remaining iteration time is " << time_iter_remain << " [ms] \n";
        // if (time_iter_remain < 0)
        //     std::cout << "*************** Real-time is broken. " << -time_iter_remain << " [ms] exceeded!!! *************** \n";

        // Update environment and check if the collision occurs
        if (!DP::motion_validity->check(DP::q_previous, DP::q_current))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "********** Robot is stopping. Collision has been occurred!!! **********");
            status = base::State::Status::Trapped;
            DP::q_next = DP::q_current;
            iteration_completed = true;
            return;     // The algorithm will still continue its execution.
        }

        // Process any invalidated nodes if obstacles have moved
        if (DP::planner_info->getNumIterations() % RRTxConfig::REPLANNING_THROTTLE == 0) {
            DP::handleDynamicObstacles();
        }

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
    DP::planner_info->setNumStates(DP::tree->getNumStates());
    // Check if we've exceeded time or iterations
    if (DP::checkTerminatingCondition(status))
    {
        if (loop_execution)
        {
            DP::q_start = q_goal_init;
            DP::q_goal = q_start_init;
            q_start_init = DP::q_start;
            q_goal_init = DP::q_goal;
        }
        else
        {
            DP::q_start = DP::q_goal;   // Robot will automatically stop and wait until 'DP::q_goal' is changed
            planning_result = DP::planner_info->getSuccessState();
        }
    }
    
    iteration_completed = true;
    // std::cout << "----------------------------------------------------------------------------------------\n";
}

/// @brief Compute trajectory points from 'spline_next' and publish them.
void sim_bringup::RealTimePlanningNode2::computeTrajectory()
{
    // Procedure of updating current state
    DP::q_next = DP::q_current->getParent();
    // std::cout << "DP::q_current: " << DP::q_current << "\n";
    // std::cout << "DP::q_next:    " << DP::q_next << "\n";
    
    status = base::State::Status::None;
    std::shared_ptr<base::State> q_current_new = DP::ss->getNewState(DP::q_current->getCoord());
    DP::updating_state->setNonZeroFinalVel(!DP::ss->isEqual(DP::q_next, DP::q_goal));
    DP::updating_state->setTimeIterStart(DP::time_iter_start);
    DP::updating_state->setMeasureTime(true);
    DP::updating_state->update(DP::q_previous, q_current_new, DP::q_next, status);

    float t_delay { DP::updating_state->getRemainingTime() };
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New trajectory is computed! Delay time: %f [ms]", t_delay * 1e3);
    if (DP::traj->spline_next != DP::traj->spline_current)  // New spline is computed
        DP::traj->spline_next->setTimeStart(t_delay);

    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    Trajectory::clear();
    Trajectory::addPoints(DP::traj->spline_next, 
                        DP::traj->spline_next->getTimeCurrent() + trajectory_advance_time, 
                        DP::traj->spline_next->getTimeEnd() + DRGBTConfig::MAX_TIME_TASK1 + trajectory_advance_time);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Elapsed time: %f [ms] for adding %ld points", 
    //             DP::getElapsedTime(time_start_) * 1e3, Trajectory::getNumPoints());

    while (DP::getElapsedTime(time_start_) < t_delay) {}    // Wait for 't_delay' to exceed...
    Trajectory::publish();

    if (status == base::State::Status::Advanced ||
        (status == base::State::Status::Reached && !DP::ss->isEqual(q_current_new, DP::q_next)))
    {
        // Update cost
        q_current_new->setCost(DP::q_next->getCost() + DP::distance(DP::q_next, q_current_new));
        
        // Find neighbors within DP::r_rewire
        std::vector<std::shared_ptr<base::State>> neighbors = DP::findNeighbors(q_current_new, DP::r_rewire);
        
        // If possible, choose parent that minimizes cost. Otherwise, remain the old parent.
        if (DP::chooseParent(q_current_new, neighbors))
            DP::tree->upgradeTree(q_current_new, q_current_new->getParent());
        else
            DP::tree->upgradeTree(q_current_new, DP::q_next);
        
        // Rewire the tree
        DP::rewireNeighbors(q_current_new, neighbors);

        DP::q_current = q_current_new;
    }
    else if (status == base::State::Status::Reached)
        DP::q_current = DP::q_next;
}
