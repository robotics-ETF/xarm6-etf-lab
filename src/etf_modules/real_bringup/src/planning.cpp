#include "planning.h"

PlanningNode::PlanningNode(const std::string scenario_file_path, const std::string node_name, const int period, 
    const std::string time_unit) : BaseNode(node_name, period, time_unit)
{
    bounding_boxes_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/bounding_boxes", 10, std::bind(&PlanningNode::boundingBoxesCallback, this, std::placeholders::_1));

    project_path = std::string(__FILE__);
    for (int i = 0; i < 3; i++)
        project_path = project_path.substr(0, project_path.find_last_of("/\\"));
    
    scenario = std::make_shared<scenario::Scenario>(scenario_file_path, project_path);
    robot = scenario->getRobot();
    env = scenario->getEnvironment();
    start = scenario->getStart();
    goal = scenario->getGoal();

    YAML::Node node = YAML::LoadFile(project_path + scenario_file_path);
    ConfigurationReader::initConfiguration(project_path + node["configurations"].as<std::string>());
    max_lin_vel = node["robot"]["max_lin_vel"].as<float>();
    max_lin_acc = node["robot"]["max_lin_acc"].as<float>();
    max_ang_vel = node["robot"]["max_ang_vel"].as<float>();
    max_ang_acc = node["robot"]["max_ang_acc"].as<float>();
}

void PlanningNode::planningCallback()
{
    switch (state)
    {
    case 0:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State 0");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home...");
        scenario->setStart(std::make_shared<base::RealVectorSpaceState>(joint_states));
        scenario->setGoal(start);
        state++;
        break;

    case 1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State 1"); 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating the environment..."); 
        updateEnvironment();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning the path..."); 
        if (planPath())
        {
            parametrizePath();
            state++;
        }
        break;
    
    case 2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State 2");
        publishTrajectory(path, path_times, 0.1);
        state = -1;
        break;

    default:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State: Executing trajectory...");
        if ((joint_states - scenario->getGoal()->getCoord()).norm() < 0.1)
        {
            scenario->setStart(start);
            scenario->setGoal(goal);

            // Swap start and goal for next motion, and repeat the procedure
            start = scenario->getGoal();
            goal = scenario->getStart();
            state = 1;
        }
        break;
    }
    RCLCPP_INFO(this->get_logger(), "----------------------------------------------------------------\n");
}

void PlanningNode::boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    bounding_boxes.clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::moveFromROSMsg(*msg, *pcl);
    for (int i = 0; i < pcl->size(); i += 2)
    {
        pcl::PointXYZ dim = pcl->points[i];
        pcl::PointXYZ pos = pcl->points[i+1];
        bounding_boxes.emplace_back(fcl::Vector3f(dim.x, dim.y, dim.z));
        bounding_boxes.emplace_back(fcl::Vector3f(pos.x, pos.y, pos.z));
        // RCLCPP_INFO(this->get_logger(), "Bounding-box %d: dim = (%f, %f, %f), pos = (%f, %f, %f)",
        //     i/2, dim.x, dim.y, dim.z, pos.x, pos.y, pos.z);
    }
}

void PlanningNode::updateEnvironment()
{
    env->removeCollisionObjects(1);  // table: idx = 0
    
    // Bounding-boxes
    Eigen::Matrix3f rot = fcl::Quaternionf(0, 0, 0, 0).matrix();
    for (int i = 0; i < bounding_boxes.size(); i += 2)
    {
        std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(bounding_boxes[i]);
        std::shared_ptr<fcl::CollisionObjectf> ob = std::make_shared<fcl::CollisionObjectf>(box, rot, bounding_boxes[i+1]);
        env->addCollisionObject(ob);
    }

    // // Predefined fixed obstacle
    // std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(fcl::Vector3f(0.1, 0.1, 0.3));
    // std::shared_ptr<fcl::CollisionObjectf> ob = std::make_shared<fcl::CollisionObjectf>(box, rot, fcl::Vector3f(0.2, 0.2, 0.15));
    // env->addCollisionObject(ob);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Environment is updated."); 
}

// Parametrize the obtained path 'planner_path' in order to satisfy the maximal angular velocity 'max_ang_vel'
void PlanningNode::parametrizePath()
{
    float time = 0;
    path_times.clear();
    path_times.emplace_back(time);

    Eigen::VectorXf q = planner_path[0]->getCoord();
    Eigen::VectorXf q_next;
    path.clear();
    path.emplace_back(q);

    for (int i = 1; i < planner_path.size(); i++)
    {
        q_next = planner_path[i]->getCoord();
        time += (q_next - q).norm() / max_ang_vel;
        path_times.emplace_back(time);
        path.emplace_back(q_next);
        q = q_next;
    }
}

// Interpolate the obtained path 'planner_path' in order to satisfy the maximal angular velocity 'max_ang_vel'
void PlanningNode::parametrizePath(float delta_time)
{
    float time = 0;
    path_times.clear();
    path_times.emplace_back(time);

    Eigen::VectorXf q = planner_path[0]->getCoord();
    Eigen::VectorXf q_next;
    path.clear();
    path.emplace_back(q);

    float max_delta_angle = max_ang_vel * delta_time;
    for (int i = 1; i < planner_path.size(); i++)
    {
        q_next = planner_path[i]->getCoord();
        float d = (q_next - q).norm();
        bool status = false;
        while (!status)
        {
            if (d > max_delta_angle)
            {
                q += ((q_next - q) / d) * max_delta_angle;
                d -= max_delta_angle;
            }
            else
            {
                q = q_next;
                status = true;
            }
            path.emplace_back(q);
            time += delta_time;
            path_times.emplace_back(time);
        }
    }
}

// If needed: DO NOT forget to set the configuration parameters for the used planner in the folder RPMPLv2/data/configurations
bool PlanningNode::planPath()
{
    bool res = false;
    std::shared_ptr<base::StateSpace> ss = scenario->getStateSpace();
    std::shared_ptr<base::State> start_ = scenario->getStart();
    std::shared_ptr<base::State> goal_ = scenario->getGoal();
    
    LOG(INFO) << "Number of collision objects: " << env->getParts().size();
    LOG(INFO) << "Dimensions: " << ss->getDimensions();
    LOG(INFO) << "State space type: " << ss->getStateSpaceType();
    LOG(INFO) << "Start: " << start_;
    LOG(INFO) << "Goal: " << goal_;

    try
    {
        // std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rrt::RRTConnect>(ss, start_, goal_);
        // std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RBTConnect>(ss, start_, goal_);
        // std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RGBTConnect>(ss, start_, goal_);
        std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, start_, goal_);
        res = planner->solve();
        LOG(INFO) << "Planning is finished with " << (res ? "SUCCESS!" : "FAILURE!");
        LOG(INFO) << "Number of states in the trees: " << planner->getPlannerInfo()->getNumStates();
        LOG(INFO) << "Number of states in the path: " << planner->getPath().size();
        LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
        LOG(INFO) << "Path cost: " << planner->getPlannerInfo()->getCostConvergence().back();   // Only for optimal planners
        planner->outputPlannerData(project_path + "/real_bringup/data/planner_data.log");

        if (res)
            planner_path = planner->getPath();
    }
    catch (std::domain_error &e)
    {
        LOG(ERROR) << e.what();
    }
    
    return res;
}
