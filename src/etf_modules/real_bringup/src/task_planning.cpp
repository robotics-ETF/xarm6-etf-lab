#include "task_planning.h"

TaskPlanningNode::TaskPlanningNode(const std::string scenario_file_path, const std::string node_name, const int period, 
    const std::string time_unit) : PlanningNode(scenario_file_path, node_name, period, time_unit)
{
    xarm_client_node = std::make_shared<rclcpp::Node>("xarm_client_node");
    xarm_client.init(xarm_client_node, "xarm");
    xarm_client.set_gripper_enable(true);
    xarm_client.set_gripper_mode(0);
    xarm_client.set_gripper_speed(3000);

    task = 0;
    IK_computed = -1;
}

void TaskPlanningNode::taskPlanningCallback()
{
    switch (task)
    {
    case 1:
        task = 0;
        chooseObject();
        if (obj_idx != -1)
        {
            task = 101;
            IK_computed = computeObjectApproachAndPickAngles();
        }
        break;

    case 2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going towards the object...");
        xarm_client.set_gripper_position(850);
        planner_path.clear();
        planner_path.emplace_back(q_object_approach1);
        planner_path.emplace_back(q_object_approach2);
        planner_path.emplace_back(q_object_pick);
        parametrizePlannerPath();
        publishTrajectory(path, path_times);
        task++;
        break;

    case 3:
        task++;
        break;

    case 4:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Picking the object...");
        xarm_client.set_gripper_position(0);
        task++;
        break;

    case 5:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Picking the object...");
        task++;
        break;

    case 6:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the object...");
        planner_path.clear();
        planner_path.emplace_back(q_object_pick);
        planner_path.emplace_back(q_object_approach1);
        parametrizePlannerPath();
        publishTrajectory(path, path_times);
        task++;
        break;

    case 7:
        float gripper_pos;
        xarm_client.get_gripper_position(&gripper_pos);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper position: %f", gripper_pos);
        if (gripper_pos < 10)  // Nothing is caught
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to pick the object again...");
            xarm_client.set_gripper_position(850);
            task = 0;      // Go from the beginning
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the object to destination...");
            scenario->setStart(q_object_approach1);
            scenario->setGoal(q_goal);
            clearMeasurements();
            task = 100;
            task_next = 8;
        }
        break;
    
    case 8:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the object...");
        xarm_client.set_gripper_position(850);
        task = 0;
        break;

    case 100:   // planning
        switch (state)
        {
        case 0:
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
        
        case 1:
            publishTrajectory(path, path_times);
            state = -1;   // Just to go to default
            break;

        default:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing trajectory...");
            if ((joint_states - scenario->getGoal()->getCoord()).norm() < 0.1)
            {
                state = 0;
                task = task_next;
            }
            break;
        }
        break;

    case 101:    // IK computing
        if (IK_computed == 1)
        {
            scenario->setStart(std::make_shared<base::RealVectorSpaceState>(joint_states));
            scenario->setGoal(q_object_approach1);
            clearMeasurements();
            task = 100;
            task_next = 2;
            IK_computed = -1;
        }
        else if (IK_computed == 0)
            task = 0;
        
        break;

    default:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the object...");
        clearMeasurements();
        if (joint_states_ready)
            task = 1;   // Do measurements
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------\n");
}

void TaskPlanningNode::boundingBoxesCallbackWithFiltering(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (task == 1 || task == 100)
        PlanningNode::boundingBoxesCallbackWithFiltering(msg);
}

void TaskPlanningNode::chooseObject()
{
    float z_max = -INFINITY;
    obj_idx = -1;
    for (int i = 0; i < objects_pos.size(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object %d. dim = (%f, %f, %f), pos = (%f, %f, %f). Num. captures %d.",
            i, objects_dim[i].x(), objects_dim[i].y(), objects_dim[i].z(), 
               objects_pos[i].x(), objects_pos[i].y(), objects_pos[i].z(), num_captures[i]);
        if (num_captures[i] >= min_num_captures && objects_pos[i].z() > z_max)
        {
            z_max = objects_pos[i].z();
            obj_idx = i;
        }
    }

    if (obj_idx != -1)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object %d is recognized at the position (%f, %f, %f).", 
            obj_idx, objects_pos[obj_idx].x(), objects_pos[obj_idx].y(), objects_pos[obj_idx].z());
}

bool TaskPlanningNode::computeObjectApproachAndPickAngles()
{
    // For approaching from above
    KDL::Vector n(objects_pos[obj_idx].x(), objects_pos[obj_idx].y(), 0); n.Normalize();
    KDL::Vector s(objects_pos[obj_idx].y(), -objects_pos[obj_idx].x(), 0); s.Normalize();
    KDL::Vector a(0, 0, -1);
    
    // For approaching by side
    // KDL::Vector n(0, 0, -1);
    // KDL::Vector s(-objects_pos[obj_idx].y(), objects_pos[obj_idx].x(), 0); s.Normalize();
    // KDL::Vector a(objects_pos[obj_idx].x(), objects_pos[obj_idx].y(), 0); a.Normalize();

    KDL::Rotation R(n, s, a);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rotation matrix: ");
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector n: (%f, %f, %f)", R(0,0), R(1,0), R(2,0));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector s: (%f, %f, %f)", R(0,1), R(1,1), R(2,1));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector a: (%f, %f, %f)", R(0,2), R(1,2), R(2,2));
    
    float fi = std::atan2(objects_pos[obj_idx].y(), objects_pos[obj_idx].x());
    float r = objects_pos[obj_idx].head(2).norm();
    float r_crit = 0.3;
    if (r < r_crit)
        r += 1.5 * objects_dim[obj_idx].head(2).norm();
    else
        r -= 1.5 * objects_dim[obj_idx].head(2).norm();

    float p_pick_z = objects_pos[obj_idx].z() + offset_z;
    if (objects_dim[obj_idx].z() > 0.14)
        p_pick_z += objects_dim[obj_idx].z() / 2 - 0.07;    // finger length is 7 [cm]

    KDL::Vector p_approach1 = KDL::Vector(r * float(cos(fi)), 
                                          r * float(sin(fi)), 
                                          p_pick_z + delta_z);
    KDL::Vector p_approach2 = KDL::Vector(r * float(cos(fi)), 
                                          r * float(sin(fi)), 
                                          p_pick_z);
    KDL::Vector p_pick = KDL::Vector(objects_pos[obj_idx].x(), 
                                     objects_pos[obj_idx].y(), 
                                     p_pick_z);
    int num = 0;
    while (num++ <= 100)
    {
        q_object_approach1 = robot->computeInverseKinematics(R, p_approach1);
        if (q_object_approach1 == nullptr)
            continue;
        
        // It is convenient for the purpose when picking objects from above
        if (r > r_crit && std::abs(q_object_approach1->getCoord(3)) < 0.1 ||
            r <= r_crit && std::abs(q_object_approach1->getCoord(3) - (-M_PI)) < 0.1)
            break;
    }
    if (q_object_approach1 == nullptr)
        return false;

    q_object_approach2 = robot->computeInverseKinematics(R, p_approach2, q_object_approach1);
    if (q_object_approach2 == nullptr || std::abs(q_object_approach1->getCoord(3) - q_object_approach2->getCoord(3)) > 0.1)
        return false;

    q_object_pick = robot->computeInverseKinematics(R, p_pick, q_object_approach2); // In order to ensure that 'q_object_pick' is relatively close to 'q_object_approach1'
    if (q_object_pick == nullptr || std::abs(q_object_approach2->getCoord(3) - q_object_pick->getCoord(3)) > 0.1)
        return false;

    KDL::Vector p_goal = KDL::Vector(goal_pos.x(),
                                     goal_pos.y(),
                                     goal_pos.z() + objects_dim[obj_idx].z());
    KDL::Vector n_goal(goal_pos.x(), goal_pos.y(), 0); n_goal.Normalize();
    KDL::Vector s_goal(goal_pos.y(), -goal_pos.x(), 0); s_goal.Normalize();
    KDL::Vector a_goal(0, 0, -1);
    KDL::Rotation R_goal(n_goal, s_goal, a_goal);
    Eigen::VectorXf goal_angles = q_object_pick->getCoord();
    goal_angles(0) = std::atan2(goal_pos.y(), goal_pos.x());
    q_goal = robot->computeInverseKinematics(R_goal, p_goal, std::make_shared<base::RealVectorSpaceState>(goal_angles));
    if (q_goal == nullptr)
        return false;
    
    // Just to take a shorter angle
    if (std::abs(std::atan2(objects_pos[obj_idx].y(), objects_pos[obj_idx].x()) - q_goal->getCoord(0)) > M_PI)
    {
        if (q_goal->getCoord(0) > 0)
            q_goal->setCoord(q_goal->getCoord(0) - 2*M_PI, 0);
        else
            q_goal->setCoord(q_goal->getCoord(0) + 2*M_PI, 0);
    }

    return true;
}

bool TaskPlanningNode::whetherToRemoveBoundingBox(Eigen::Vector3f &object_pos, Eigen::Vector3f &object_dim)
{
    // Filter the destination box from the scene
    if (object_pos.x() < -0.4 && std::abs(object_pos.y()) < 0.2 && object_pos.z() < 0.25)
        return true;
    
    return false;
}