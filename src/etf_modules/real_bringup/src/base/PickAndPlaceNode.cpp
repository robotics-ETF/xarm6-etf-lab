#include "base/PickAndPlaceNode.h"

real_bringup::PickAndPlaceNode::PickAndPlaceNode(const std::string node_name, const std::string config_file_path) : 
    MoveXArm6Node(node_name, config_file_path) 
{
    Robot::xarm_client.set_mode(0);
    Robot::xarm_client.set_state(0);

    for (int i = 0; i < Robot::getHomeJointsState()->getCoord().size(); i++)
        home_angles.emplace_back(Robot::getHomeJointsState()->getCoord(i));

    task = approaching_to_object;
}

void real_bringup::PickAndPlaceNode::pickAndPlaceCallback()
{
    switch (task)
    {
    case going_home:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home..."); 
        Robot::xarm_client.set_servo_angle(home_angles, Robot::getMaxAngVel(), Robot::getMaxAngAcc(), 0, true, 1, -1);
        Robot::xarm_client.set_gripper_position(850, true, 2);
        num = 0;
        object_angles_approach[0] += delta_theta1;
        delta_theta1 *= -1;
        task = approaching_to_object;
        break;

    case approaching_to_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Approaching to the object..."); 
        Robot::xarm_client.set_servo_angle(object_angles_approach, Robot::getMaxAngVel(), Robot::getMaxAngAcc(), 0, true, 1, -1);
        task = lowering_robot;
        break;
    
    case lowering_robot:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lowering the robot...");
        Robot::xarm_client.get_position(current_pose);
        current_pose[2] = object_pick_z - num * object_height;
        Robot::xarm_client.set_position(current_pose, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, true, 1);
        task = grasping_object;
        break;

    case grasping_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasping the object..."); 
        Robot::xarm_client.set_gripper_position(400, true, 2);
        task = raising_object;
        break;

    case raising_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the object...");
        current_pose[2] = 3*object_pick_z;
        Robot::xarm_client.set_position(current_pose, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, true, 1);
        task = moving_object_to_goal;
        break;

    case moving_object_to_goal:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the object to the goal...");
        Robot::xarm_client.get_servo_angle(current_angles);
        current_angles[0] += delta_theta1;
        Robot::xarm_client.set_servo_angle(current_angles, Robot::getMaxAngVel(), Robot::getMaxAngAcc(), 0, true, 1, -1);
        task = lowering_object;
        break;

    case lowering_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lowering the object..."); 
        Robot::xarm_client.get_position(current_pose);
        current_pose[2] = object_pick_z - (2-num) * object_height;
        Robot::xarm_client.set_position(current_pose, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, true, 1);
        task = releasing_object;
        break;

    case releasing_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the object..."); 
        Robot::xarm_client.set_gripper_position(850, true, 2);
        task = raising_robot;
        break;

    case raising_robot:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the robot...");
        current_pose[2] = 3*object_pick_z;
        Robot::xarm_client.set_position(current_pose, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, true, 1);
        if (++num < num_objects)
            task = approaching_to_object;
        else
            task = going_home;     // Repeat the sequence for the next object
        break;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------------------------------"); 
}
