#include "task_planning.h"

TaskPlanningNode::TaskPlanningNode(const std::string scenario_file_path, const std::string node_name, const int period, 
    const std::string time_unit) : PlanningNode(scenario_file_path, node_name, period, time_unit)
{
    object_angles_approach << 90, 33, -96, 0, 63, 0;
    object_angles_approach *= M_PI / 180;
    task = 0;

    xarm_client_node = std::make_shared<rclcpp::Node>("xarm_client_node");
    xarm_client.init(xarm_client_node, "xarm");
    xarm_client.set_gripper_enable(true);
    xarm_client.set_gripper_mode(0);
    xarm_client.set_gripper_speed(2000);
}

void TaskPlanningNode::taskPlanningCallback()
{
    switch (task)
    {
    case 0:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task 0");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home...");
        scenario->setStart(std::make_shared<base::RealVectorSpaceState>(joint_states));
        scenario->setGoal(start);
        task = -1;
        task_next = 1;
        break;

    case 1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task 1");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going towards the object..."); 
        scenario->setStart(start);
        scenario->setGoal(std::make_shared<base::RealVectorSpaceState>(object_angles_approach));
        xarm_client.set_gripper_position(850, true, 1);
        task = -1;
        task_next = 2;
        break;
    
    case 2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task 2");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasping the object..."); 
        xarm_client.set_gripper_position(0, true, 1);
        task++;
        break;

    case 3:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task 3");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the object to the goal...");
        scenario->setStart(std::make_shared<base::RealVectorSpaceState>(joint_states));
        object_angles_approach(0) += M_PI_2;
        scenario->setGoal(std::make_shared<base::RealVectorSpaceState>(object_angles_approach));
        task = -1;
        task_next = 4;
        break;

    case 4:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task 4"); 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the object..."); 
        xarm_client.set_gripper_position(850, true, 1);
        task = 0;
        break;

    default:
        switch (state)
        {
        case 0:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State 0"); 
            updateEnvironment();

            if (planPath())
            {
                parametrizePath();
                state++;
            }
            break;
        
        case 1:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State 1");
            publishTrajectory(path, path_times, 0.1);
            state = -1;
            break;

        default:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State 2: Executing trajectory...");
            if ((joint_states - scenario->getGoal()->getCoord()).norm() < 0.1)
            {
                state = 0;
                task = task_next;
            }
            break;
        }
        break;
    }
    RCLCPP_INFO(this->get_logger(), "----------------------------------------------------------------\n");
}
