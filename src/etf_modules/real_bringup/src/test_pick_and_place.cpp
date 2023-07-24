#include "test_pick_and_place.h"

TestPickAndPlaceNode::TestPickAndPlaceNode() : Node("test_pick_and_place_node")
{
    timer = this->create_wall_timer(std::chrono::seconds(period), std::bind(&TestPickAndPlaceNode::timerCallback, this));
    xarm_client_node = std::make_shared<rclcpp::Node>("xarm_client_node");
    xarm_client.init(xarm_client_node, "xarm");    
    xarm_client.clean_error();
    xarm_client.clean_warn();
    xarm_client.motion_enable(true);
    xarm_client.set_mode(0);
    xarm_client.set_state(0);
    xarm_client.set_gripper_enable(true);
    xarm_client.set_gripper_mode(0);
    xarm_client.set_gripper_speed(2000);
}

void TestPickAndPlaceNode::timerCallback()
{
    switch (state)
    {
    case 0:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home..."); 
        xarm_client.set_servo_angle(home_angles, max_ang_vel, max_ang_acc, 0, true, 1, -1);
        xarm_client.set_gripper_position(850, true, 2);
        state++;
        break;

    case 1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 1"); 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Approaching to the object..."); 
        xarm_client.set_servo_angle(object_angles_approach, max_ang_vel, max_ang_acc, 0, true, 1, -1);
        state++;
        break;
    
    case 2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 2"); 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lowering the robot...");
        xarm_client.get_position(current_pose);
        current_pose[2] = object_pick_z - k * object_height;
        xarm_client.set_position(current_pose, -1, 0.7*max_lin_vel, 0.7*max_lin_acc, 0, true, 1);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasping the object..."); 
        xarm_client.set_gripper_position(400, true, 2);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the object...");
        current_pose[2] = 3*object_pick_z;
        xarm_client.set_position(current_pose, -1, 0.7*max_lin_vel, 0.7*max_lin_acc, 0, true, 1);
        state++;
        break;

    case 3:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 3");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the object to the goal...");
        xarm_client.get_servo_angle(current_angles);
        current_angles[0] += delta_theta1;
        xarm_client.set_servo_angle(current_angles, max_ang_vel, max_ang_acc, 0, true, 1, -1);
        state++;
        break;

    case 4:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 4"); 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lowering the object..."); 
        xarm_client.get_position(current_pose);
        current_pose[2] = object_pick_z - (2-k) * object_height;
        xarm_client.set_position(current_pose, -1, 0.7*max_lin_vel, 0.7*max_lin_acc, 0, true, 1);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the object..."); 
        xarm_client.set_gripper_position(850, true, 2);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the robot...");
        current_pose[2] = 3*object_pick_z;
        xarm_client.set_position(current_pose, -1, 0.7*max_lin_vel, 0.7*max_lin_acc, 0, true, 1);
        state++;
        break;
    
    default:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home..."); 
        xarm_client.set_servo_angle(home_angles, max_ang_vel, max_ang_acc, 0, true, 1, -1);
        xarm_client.set_gripper_position(850, true, 2);
        state = 0;
        k = 0;
        object_angles_approach[0] += delta_theta1;
        delta_theta1 *= -1;
        break;
    }

    if (state > 4 && k < 2)   // Repeat the sequence for the next object
    {
        state = 1;
        k++;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------------------------------"); 
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPickAndPlaceNode>());
    rclcpp::shutdown();
    return 0;
}