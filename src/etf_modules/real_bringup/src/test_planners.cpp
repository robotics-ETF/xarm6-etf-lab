#include "test_planners.h"

TestPlannersNode::TestPlannersNode() : Node("test_planners_node")
{
    timer = this->create_wall_timer(std::chrono::seconds(period), std::bind(&TestPlannersNode::testPlannersCallback, this));
    trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
    marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/occupied_cells_vis_array", 10);
    
    joint_states_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
        ("/xarm6_traj_controller/state", 10, std::bind(&TestPlannersNode::jointStatesCallback, this, std::placeholders::_1));
    bounding_boxes_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/bounding_boxes", 10, std::bind(&TestPlannersNode::boundingBoxesCallback, this, std::placeholders::_1));

    
    project_path = std::string(__FILE__);
    for (int i = 0; i < 3; i++)
        project_path = project_path.substr(0, project_path.find_last_of("/\\"));
    
    scenario = std::make_shared<scenario::Scenario>(scenario_file_path, project_path);
    robot = scenario->getRobot();
    env = scenario->getEnvironment();

    YAML::Node node = YAML::LoadFile(scenario_file_path);
    ConfigurationReader::initConfiguration(node["configurations"].as<std::string>());
    trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    state = 0;
}

void TestPlannersNode::testPlannersCallback()
{
    switch (state)
    {
    case 0:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 0"); 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating the environment..."); 
        updateEnvironment();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning the path..."); 
        if (planPath())
        {
            parametrizePath(max_ang_vel);
            state++;
        }
        break;
    
    case 1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 1");
        publishTrajectory(0.1);
        state = -1;
        break;

    default:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case: Executing trajectory...");
        if ((robot->getConfiguration()->getCoord() - scenario->getGoal()->getCoord()).norm() < 1e-2)
        {
            // Swap start and goal, and repeat the procedure
            std::shared_ptr<base::State> start_ = scenario->getStart();
            std::shared_ptr<base::State> goal_ = scenario->getGoal();
            scenario->setStart(goal_);
            scenario->setGoal(start_);
            state = 0;
        }
        break;
    }
    RCLCPP_INFO(this->get_logger(), "----------------------------------------------------------------\n");
}

void TestPlannersNode::jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    std::vector<double> positions = msg->actual.positions;
    Eigen::VectorXf q(6);
    q << positions[0], positions[1], positions[2], positions[3], positions[4], positions[5];
    robot->setConfiguration(scenario->getStateSpace()->newState(q));
    // RCLCPP_INFO(this->get_logger(), "Robot joint states: (%f, %f, %f, %f, %f, %f).", q(0), q(1), q(2), q(3), q(4), q(5));
}

void TestPlannersNode::boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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

void TestPlannersNode::updateEnvironment()
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
}

// Parametrize the obtained path 'planner_path' in order to satisfy the maximal angular velocity 'max_ang_vel'
void TestPlannersNode::parametrizePath(float max_ang_vel)
{
    path_times.clear();
    float time = 0;
    path_times.emplace_back(time);
    for (int i = 0; i < planner_path.size()-1; i++)
    {
        Eigen::VectorXf q = planner_path[i]->getCoord();
        Eigen::VectorXf q_next = planner_path[i+1]->getCoord();
        time += (q_next - q).norm() / max_ang_vel;
        path_times.emplace_back(time);
    }
}

void TestPlannersNode::publishTrajectory(float init_time)
{
    if (planner_path.empty())
    {
        RCLCPP_INFO(this->get_logger(), "There is no trajectory to publish!\n");
        return;
    }
    
    trajectory.points.clear();

    RCLCPP_INFO(this->get_logger(), "Trajectory: ");
    for (int i = 0; i < planner_path.size(); i++)
    {
        Eigen::VectorXf q = planner_path[i]->getCoord();
        RCLCPP_INFO(this->get_logger(), "Num. %d. Time: %f [s]. Point: (%f, %f, %f, %f, %f, %f)", 
            i, path_times[i] + init_time, q(0), q(1), q(2), q(3), q(4), q(5));

        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (int j = 0; j < q.size(); j++)
            point.positions.emplace_back(q(j));

        point.time_from_start.sec = int32_t(path_times[i] + init_time);
        point.time_from_start.nanosec = (path_times[i] + init_time - point.time_from_start.sec) * 1e9;
        trajectory.points.emplace_back(point);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing the trajectory ...\n");
    trajectory_publisher->publish(trajectory);
}

bool TestPlannersNode::planPath()
{
    bool res = false;
    std::shared_ptr<base::StateSpace> ss = scenario->getStateSpace();
    std::shared_ptr<base::State> start = scenario->getStart();
    std::shared_ptr<base::State> goal = scenario->getGoal();
    
    LOG(INFO) << "Number of collision objects: " << env->getParts().size();
    LOG(INFO) << "Dimensions: " << ss->getDimensions();
    LOG(INFO) << "State space type: " << ss->getStateSpaceType();
    LOG(INFO) << "Start: " << start;
    LOG(INFO) << "Goal: " << goal;

    try
    {
        // std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rrt::RRTConnect>(ss, start, goal);
        // std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RBTConnect>(ss, start, goal);
        // std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RGBTConnect>(ss, start, goal);
        std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, start, goal);
        res = planner->solve();
        LOG(INFO) << "Planning is finished with " << (res ? "SUCCESS!" : "FAILURE!");
        LOG(INFO) << "Number of states in the trees: " << planner->getPlannerInfo()->getNumStates();
        LOG(INFO) << "Number of states in the path: " << planner->getPath().size();
        LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
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

int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPlannersNode>());
    rclcpp::shutdown();
	google::ShutDownCommandLineFlags();
    return 0;
}