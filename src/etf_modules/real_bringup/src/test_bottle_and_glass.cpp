#include "test_bottle_and_glass.h"

TestBottleAndGlassNode::TestBottleAndGlassNode() : Node("test_bottle_and_glass_node")
{
    timer = this->create_wall_timer(std::chrono::seconds(period), std::bind(&TestBottleAndGlassNode::timerCallback, this));
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

void TestBottleAndGlassNode::timerCallback()
{
    switch (state)
    {
    case 1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 1"); 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Approaching to the bottle..."); 
        xarm_client.set_position(bottle_pose_pick, -1, 0.5*max_lin_vel, 0.5*max_lin_acc, 0, true, 1);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasping the bottle..."); 
        xarm_client.set_gripper_position(500, true, 2);

        state++;
        break;

    case 2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 2");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the bottle..."); 
        current_pose = bottle_pose_pick;
        current_pose[2] = 200;
        xarm_client.set_position(current_pose, -1, 0.5*max_lin_vel, 0.5*max_lin_acc, 0, true, 1);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving towards the glass..."); 
        xarm_client.get_servo_angle(current_angles);
        current_angles[0] += sign * M_PI_4;
        xarm_client.set_servo_angle(current_angles, max_ang_vel, max_ang_acc, 0, true, 1, -1);
        state++;
        break;

    case 3:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 3");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pouring the watter into the glass..."); 
        current_angles[5] -= sign * M_PI_2;
        xarm_client.set_servo_angle(current_angles, 0.7*max_ang_vel, 0.7*max_ang_acc, 0, true, 1, -1);
        state++;
        break;
    
    case 4:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 4");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pouring the watter into the glass..."); 
        current_angles[5] += sign * M_PI_2;
        xarm_client.set_servo_angle(current_angles, 0.7*max_ang_vel, 0.7*max_ang_acc, 0, true, 1, -1);
        state++;
        break;

    case 5:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 5");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Returning the bottle..."); 
        xarm_client.get_servo_angle(current_angles);
        current_angles[0] -= sign * M_PI_4;
        xarm_client.set_servo_angle(current_angles, max_ang_vel, max_ang_acc, 0, true, 1, -1);
        xarm_client.set_position(bottle_pose_pick, -1, max_lin_vel, max_lin_acc, 0, true, 1);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the bottle...");
        xarm_client.set_gripper_position(850, true, 2);
        state++;
        break;
    
    case 6:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 6: Going home..."); 
        xarm_client.set_servo_angle(home_angles, max_ang_vel, max_ang_acc, 0, true, 1, -1);
        state++;
        break;
    
    case 7:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 7");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving towards the glass...");
        xarm_client.set_servo_angle(glass_angles_approach, max_ang_vel, max_ang_acc, 0, true, 1, -1);
        xarm_client.set_servo_angle(glass_angles_pick, 0.5*max_ang_vel, 0.5*max_ang_acc, 0, true, 1, -1);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasping the glass...");
        xarm_client.set_gripper_position(550, true, 2);
        state++;
        break;

    case 8:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 8");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the glass...");
        current_pose = bottle_pose_pick;
        current_pose[2] *= 5;
        xarm_client.set_position(current_pose, -1, max_lin_vel, max_lin_acc, 0, true, 1);
        state++;
        break;
    
    case 9:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Case 9");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the glass...");
        glass_angles_approach[0] *= -1;
        glass_angles_pick[0] *= -1;
        sign *= -1;
        xarm_client.set_servo_angle(glass_angles_pick, max_ang_vel, max_ang_acc, 0, true, 1, -1);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the glass...");
        xarm_client.set_gripper_position(850, true, 2);
        xarm_client.set_servo_angle(glass_angles_approach, max_ang_vel, max_ang_acc, 0, true, 1, -1);
        state++;
        break;

    default:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home..."); 
        xarm_client.set_servo_angle(home_angles, max_ang_vel, max_ang_acc, 0, true, 1, -1);
        xarm_client.set_gripper_position(850, true, 2);
        state = 1;
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------------------------------"); 
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestBottleAndGlassNode>());
    rclcpp::shutdown();
    return 0;
}