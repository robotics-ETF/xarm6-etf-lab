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
    convex_hulls_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/convex_hulls", 10, std::bind(&TestPlannersNode::convexHullsCallback, this, std::placeholders::_1));
    convex_hulls_polygons_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/convex_hulls_polygons", 10, std::bind(&TestPlannersNode::convexHullsPolygonsCallback, this, std::placeholders::_1));

    read_octomap_node = std::make_shared<rclcpp::Node>("read_octomap_node");
    octomap_client = this->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");

    project_path = std::string(__FILE__);
    for (int i = 0; i < 3; i++)
        project_path = project_path.substr(0, project_path.find_last_of("/\\"));
    
    scenario = std::make_shared<scenario::Scenario>(scenario_file_path, project_path);
    robot = scenario->getRobot();
    env = scenario->getEnvironment();

    YAML::Node node = YAML::LoadFile(project_path + scenario_file_path);
    ConfigurationReader::initConfiguration(project_path + node["configurations"].as<std::string>());
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
        if ((robot->getConfiguration()->getCoord() - scenario->getGoal()->getCoord()).norm() < 0.1)
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

void TestPlannersNode::convexHullsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    convex_hulls = {std::vector<fcl::Vector3f>()};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::moveFromROSMsg(*msg, *pcl);

    int j = 0;
    for (int i = 0; i < pcl->size(); i++)
    {
        pcl::PointXYZRGB P = pcl->points[i];
        if (P.x == 0.0 && P.y == 0.0 && P.z == 0.0)     // This point is just delimiter to distinguish clusters
        {
            convex_hulls.emplace_back(std::vector<fcl::Vector3f>());
            j++;
        }
        else
        {
            convex_hulls[j].emplace_back(fcl::Vector3f(P.x, P.y, P.z));
            // RCLCPP_INFO(this->get_logger(), "Convex-hull %d.\t Point: (%f, %f, %f)", j, P.x, P.y, P.z);
        }
    }    
}

void TestPlannersNode::convexHullsPolygonsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    convex_hulls_polygons = {std::vector<int>()};
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::moveFromROSMsg(*msg, *pcl);

    int j = 0;
    for (int i = 0; i < pcl->size(); i++)
    {
        pcl::PointXYZ P = pcl->points[i];
        if (P.x == -1)     // This point is just delimiter to distinguish clusters
        {
            convex_hulls_polygons.emplace_back(std::vector<int>());
            j++;
        }
        else
        {
            convex_hulls_polygons[j].emplace_back(P.x);
            convex_hulls_polygons[j].emplace_back(P.y);
            convex_hulls_polygons[j].emplace_back(P.z);
            // RCLCPP_INFO(this->get_logger(), "Convex-hull %d.\t Polygon indices: (%f, %f, %f)", j, P.x, P.y, P.z);
        }
    }    
}

void TestPlannersNode::readOctomap()
{    
    while (!octomap_client->wait_for_service(1s)) 
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    // Wait for the result  
    auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
    auto result = octomap_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(read_octomap_node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        octomap_msgs::msg::Octomap octomap_msg = result.get()->map;
        octomap::AbstractOcTree* octomap_abstract_octree = octomap_msgs::binaryMsgToMap(octomap_msg);
        octomap_octree = dynamic_cast<octomap::OcTree*>(octomap_abstract_octree);

        fcl::OcTreef Octree(std::make_shared<const octomap::OcTree>(*octomap_octree));
        octree = std::make_shared<fcl::OcTreef>(Octree);            
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Octree read successfully!");
        visualizeOctreeBoxes();
    }
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to read octree!");
}

void TestPlannersNode::visualizeOctreeBoxes()
{
    std::vector<std::array<float, 6>> boxes = octree->toBoxes();
    visualization_msgs::msg::MarkerArray marker_array_msg;
    for (int i = 0; i < boxes.size(); i++) 
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Box %d: (%f, %f, %f)", i, boxes[i][0], boxes[i][1], boxes[i][2]);
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "octree_boxes";
        marker.id = i;
        marker.header.frame_id = "world";
        marker.header.stamp = now();
        marker.pose.position.x = boxes[i][0];
        marker.pose.position.y = boxes[i][1];
        marker.pose.position.z = boxes[i][2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker_array_msg.markers.emplace_back(marker);
    }
    marker_array_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing octree boxes...");
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

    // Predefined fixed obstacle
    std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(fcl::Vector3f(0.1, 0.1, 0.3));
    std::shared_ptr<fcl::CollisionObjectf> ob = std::make_shared<fcl::CollisionObjectf>(box, rot, fcl::Vector3f(0.2, 0.2, 0.15));
    env->addCollisionObject(ob);
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

// Interpolate the obtained path 'planner_path' in order to satisfy the maximal angular velocity 'max_ang_vel'
// void TestPlannersNode::parametrizePath(float max_ang_vel)
// {
//     path.clear();
//     float max_delta_angle = max_ang_vel * delta_time;
//     for (int i = 0; i < planner_path.size()-1; i++)
//     {
//         Eigen::VectorXf q = planner_path[i]->getCoord();
//         Eigen::VectorXf q_next = planner_path[i+1]->getCoord();
//         float d = (q_next - q).norm();
//         bool status = false;
//         while (!status)
//         {
//             if (d > max_delta_angle)
//             {
//                 q += ((q_next - q) / d) * max_delta_angle;
//                 d -= max_delta_angle;
//             }
//             else
//             {
//                 q = q_next;
//                 status = true;
//             }
//             path.emplace_back(q);
//         }
//     }
// }

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
        planner->outputPlannerData(project_path + "/sim_bringup/data/planner_data.log");

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