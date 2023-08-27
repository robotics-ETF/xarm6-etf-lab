#include "planning.h"

PlanningNode::PlanningNode(const std::string scenario_file_path, const std::string node_name, const int period, 
    const std::string time_unit) : BaseNode(node_name, period, time_unit)
{
    // bounding_boxes_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
    //     ("/bounding_boxes", 10, std::bind(&PlanningNode::boundingBoxesCallback, this, std::placeholders::_1));
    bounding_boxes_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/bounding_boxes", 10, std::bind(&PlanningNode::boundingBoxesCallbackWithFiltering, this, std::placeholders::_1));

    project_path = std::string(__FILE__);
    for (int i = 0; i < 3; i++)
        project_path = project_path.substr(0, project_path.find_last_of("/\\"));
    
    scenario = std::make_shared<scenario::Scenario>(scenario_file_path, project_path);
    robot = scenario->getRobot();
    env = scenario->getEnvironment();
    start = scenario->getStart();
    goal = scenario->getGoal();
    planner_ready = true;

    YAML::Node node = YAML::LoadFile(project_path + scenario_file_path);
    max_lin_vel = node["robot"]["max_lin_vel"].as<float>();
    max_lin_acc = node["robot"]["max_lin_acc"].as<float>();
    max_ang_vel = node["robot"]["max_ang_vel"].as<float>();
    max_ang_acc = node["robot"]["max_ang_acc"].as<float>();

    ConfigurationReader::initConfiguration(project_path + node["configurations"].as<std::string>());
    min_num_captures = node["min_num_captures"].as<int>();    

    planner_name = node["planner"]["name"].as<std::string>();
    if (planner_name == "RRTConnect")
        RRTConnectConfig::MAX_PLANNING_TIME = node["planner"]["max_planning_time"].as<float>();
    else if (planner_name == "RBTConnect")
        RBTConnectConfig::MAX_PLANNING_TIME = node["planner"]["max_planning_time"].as<float>();
    else if (planner_name == "RGBTConnect")
        RGBTConnectConfig::MAX_PLANNING_TIME = node["planner"]["max_planning_time"].as<float>();
    else if (planner_name == "RGBMT*")
        RGBMTStarConfig::MAX_PLANNING_TIME = node["planner"]["max_planning_time"].as<float>();
}

void PlanningNode::planningCallback()
{
    switch (state)
    {
    case 0:
        if (joint_states_ready)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home...");        
            scenario->setStart(std::make_shared<base::RealVectorSpaceState>(joint_states));
            scenario->setGoal(start);
            state++;
        }
        break;

    case 1:
        if (planner_ready)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating the environment..."); 
            updateEnvironment();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning the path..."); 
            if (planPath())
            {
                parametrizePlannerPath();
                state++;
            }
        }
        else
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the planner..."); 
        break;
    
    case 2:
        publishTrajectory(path, path_times);
        state = -1;   // Just to go to default
        break;

    default:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing trajectory...");
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------\n");
}

void PlanningNode::boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    objects_dim.clear();
    objects_pos.clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::moveFromROSMsg(*msg, *pcl);
    for (int i = 0; i < pcl->size(); i += 2)
    {
        pcl::PointXYZ dim = pcl->points[i];
        pcl::PointXYZ pos = pcl->points[i+1];
        objects_dim.emplace_back(fcl::Vector3f(dim.x, dim.y, dim.z));
        objects_pos.emplace_back(fcl::Vector3f(pos.x, pos.y, pos.z));
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bounding-box %d: dim = (%f, %f, %f), pos = (%f, %f, %f)",
        //     i/2, dim.x, dim.y, dim.z, pos.x, pos.y, pos.z);
    }
}

void PlanningNode::boundingBoxesCallbackWithFiltering(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
    pcl::moveFromROSMsg(*msg, *pcl);
    Eigen::Vector3f new_object_pos;
    Eigen::Vector3f new_object_dim;
    bool found = false;

    for (int i = 0; i < pcl->size(); i += 2)
    {
        new_object_dim << pcl->points[i].x, pcl->points[i].y, pcl->points[i].z;
        new_object_pos << pcl->points[i+1].x, pcl->points[i+1].y, pcl->points[i+1].z;

        if (whetherToRemoveBoundingBox(new_object_pos, new_object_dim))
            continue;

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bounding-box %d: dim = (%f, %f, %f), pos = (%f, %f, %f)",  // (x, y, z) in [m]
        //     i/2, new_object_dim.x(), new_object_dim.y(), new_object_dim.z(), new_object_pos.x(), new_object_pos.y(), new_object_pos.z());
    
        // Measurements are averaged online
        for (int j = 0; j < objects_dim.size(); j++)
        {
            if ((new_object_dim - objects_dim[j]).norm() < 0.05 && (new_object_pos - objects_pos[j]).norm() < 0.05)
            {
                objects_dim[j] = (num_captures[j] * objects_dim[j] + new_object_dim) / (num_captures[j] + 1);
                objects_pos[j] = (num_captures[j] * objects_pos[j] + new_object_pos) / (num_captures[j] + 1);
                num_captures[j]++;
                found = true;
                break;
            }
        }
        
        if (!found)
        {
            objects_dim.emplace_back(new_object_dim);
            objects_pos.emplace_back(new_object_pos);
            num_captures.emplace_back(1);
        }                
    }   
}

bool PlanningNode::whetherToRemoveBoundingBox(Eigen::Vector3f &object_pos, Eigen::Vector3f &object_dim)
{    
    return false;
}

void PlanningNode::updateEnvironment()
{
    env->removeCollisionObjects(1);  // table (idx = 0) is preserved, and all other objects are deleted
    
    // Bounding-boxes
    Eigen::Matrix3f rot = fcl::Quaternionf(0, 0, 0, 0).matrix();
    for (int i = 0; i < objects_pos.size(); i++)
    {
        std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(objects_dim[i]);
        std::shared_ptr<fcl::CollisionObjectf> ob = std::make_shared<fcl::CollisionObjectf>(box, rot, objects_pos[i]);
        env->addCollisionObject(ob);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bounding-box %d: dim = (%f, %f, %f), pos = (%f, %f, %f), num. captures = %d",
            i, objects_dim[i].x(), objects_dim[i].y(), objects_dim[i].z(),                      // (x, y, z) in [m]
               objects_pos[i].x(), objects_pos[i].y(), objects_pos[i].z(), num_captures[i]);    // (x, y, z) in [m]
    }

    // // Predefined fixed obstacle
    // std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(fcl::Vector3f(0.1, 0.1, 0.3));
    // std::shared_ptr<fcl::CollisionObjectf> ob = std::make_shared<fcl::CollisionObjectf>(box, rot, fcl::Vector3f(0.2, 0.2, 0.15));
    // env->addCollisionObject(ob);

    clearMeasurements();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Environment is updated."); 
}

void PlanningNode::clearMeasurements()
{
    objects_pos.clear();
    objects_dim.clear();
    num_captures.clear();
}

// Parametrize the obtained path 'planner_path' in order to satisfy the maximal angular velocity 'max_ang_vel'
void PlanningNode::parametrizePlannerPath()
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
void PlanningNode::parametrizePlannerPath(float delta_time)
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

bool PlanningNode::planPath()
{
    planner_ready = false;
    bool res = false;
    std::shared_ptr<base::StateSpace> ss = scenario->getStateSpace();
    std::shared_ptr<base::State> start_ = scenario->getStart();
    std::shared_ptr<base::State> goal_ = scenario->getGoal();
    
    LOG(INFO) << "Number of collision objects: " << env->getParts().size();
    LOG(INFO) << "Number of DOFs: " << ss->getNumDimensions();
    LOG(INFO) << "State space type: " << ss->getStateSpaceType();
    LOG(INFO) << "Start: " << start_;
    LOG(INFO) << "Goal: " << goal_;

    try
    {
        std::unique_ptr<planning::AbstractPlanner> planner;
        if (planner_name == "RRTConnect")
            planner = std::make_unique<planning::rrt::RRTConnect>(ss, start_, goal_);
        else if (planner_name == "RBTConnect")
            planner = std::make_unique<planning::rbt::RBTConnect>(ss, start_, goal_);
        else if (planner_name == "RGBTConnect")
            planner = std::make_unique<planning::rbt::RGBTConnect>(ss, start_, goal_);
        else if (planner_name == "RGBMT*")
            planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, start_, goal_);
        
        res = planner->solve();
        LOG(INFO) << planner_name << " planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
        LOG(INFO) << "Number of states in the trees: " << planner->getPlannerInfo()->getNumStates();
        LOG(INFO) << "Number of states in the path: " << planner->getPath().size();
        LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
        if (planner_name == "RGBMT*")
            LOG(INFO) << "Path cost: " << planner->getPlannerInfo()->getCostConvergence().back();
        planner->outputPlannerData(project_path + "/real_bringup/data/planner_data.log");

        if (res)
            planner_path = planner->getPath();
    }
    catch (std::domain_error &e)
    {
        LOG(ERROR) << e.what();
    }

    planner_ready = true;
    return res;
}
