#include "base/PickAndPlaceUsingCamerasNode.h"

real_bringup::PickAndPlaceUsingCamerasNode::PickAndPlaceUsingCamerasNode(const std::string node_name, const std::string config_file_path) : 
    PickAndPlaceNode(node_name, config_file_path),
    AABB(config_file_path)
{
    AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/bounding_boxes", 10, std::bind(&AABB::withFilteringCallback, this, std::placeholders::_1));

    task = waiting_for_object;
}

void real_bringup::PickAndPlaceUsingCamerasNode::pickAndPlaceUsingCamerasCallback()
{
    // Robot::testOrientation();

    switch (task)
    {
    case waiting_for_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the object...");
        AABB::resetMeasurements();
        if (Robot::isReady())
            task = choosing_object;
        break;

    case choosing_object:
        obj_idx = chooseObject();
        if (obj_idx != -1)
        {
            task = going_towards_object;
            computeObjectApproachAndPickPose();
        }
        else
            task = waiting_for_object;
        break;

    case going_towards_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going towards the object...");
        Robot::xarm_client.get_servo_angle(current_angles);
        current_angles[0] = std::atan2(AABB::getPositions(obj_idx).y(), AABB::getPositions(obj_idx).x());            
        Robot::xarm_client.set_servo_angle(current_angles, Robot::getMaxAngVel(), Robot::getMaxAngAcc(), 0, true, 1, -1);
        Robot::xarm_client.set_position(object_approach_pose, -1, Robot::getMaxLinVel(), Robot::getMaxLinAcc(), 0, true, 1);
        Robot::xarm_client.set_gripper_position(850, true, 1);
        task = picking_object;
        break;

    case picking_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Picking the object...");
        current_pose = object_approach_pose; 
        current_pose[2] -= delta_z;
        Robot::xarm_client.set_position(current_pose, -1, 0.5*Robot::getMaxLinVel(), 0.5*Robot::getMaxLinAcc(), 0, true, 1);
        Robot::xarm_client.set_position(object_pick_pose, -1, 0.5*Robot::getMaxLinVel(), 0.5*Robot::getMaxLinAcc(), 0, true, 1);
        Robot::xarm_client.set_gripper_position(0, true, 1);
        task = raising_object;
        break;

    case raising_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the object...");
        current_pose[2] += 2*delta_z;
        Robot::xarm_client.set_position(current_pose, -1, Robot::getMaxLinVel(), Robot::getMaxLinAcc(), 0, true, 1);
        task = moving_object_to_destination;
        break;

    case moving_object_to_destination:
        float gripper_pos;
        xarm_client.get_gripper_position(&gripper_pos);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper position: %f", gripper_pos);
        if (gripper_pos < 10)   // Nothing is caught
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to pick the object again...");
            xarm_client.set_gripper_position(850);
            task = waiting_for_object;     // Go from the beginning
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the object to destination...");
            Robot::xarm_client.get_servo_angle(current_angles);
            if (std::atan2(AABB::getPositions(obj_idx).y(), AABB::getPositions(obj_idx).x()) > 0)
                current_angles[0] = M_PI - 0.05;
            else
                current_angles[0] = -M_PI + 0.05;
            
            Robot::xarm_client.set_servo_angle(current_angles, Robot::getMaxAngVel(), Robot::getMaxAngAcc(), 0, true, 1, -1);
            task = releasing_object;
            offset_z = 0;
        }
        break;
    
    case releasing_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the object...");       
        Robot::xarm_client.get_position(current_pose);
        current_pose[0] = -550;
        Robot::xarm_client.set_position(current_pose, -1, 0.5*Robot::getMaxLinVel(), 0.5*Robot::getMaxLinAcc(), 0, true, 1);
        Robot::xarm_client.set_gripper_position(850, true, 1);
        task = waiting_for_object;
        break;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------------------------------"); 
}

const int real_bringup::PickAndPlaceUsingCamerasNode::chooseObject()
{
    float z_max = -INFINITY;
    int obj_idx = -1;
    for (int i = 0; i < positions.size(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object %d. dim = (%f, %f, %f), pos = (%f, %f, %f). Num. captures %d.",
            i, dimensions[i].x(), dimensions[i].y(), dimensions[i].z(), 
               positions[i].x(), positions[i].y(), positions[i].z(), num_captures[i]);
        if (num_captures[i] >= min_num_captures && positions[i].z() > z_max)
        {
            z_max = positions[i].z();
            obj_idx = i;
        }
    }

    if (obj_idx != -1)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object %d is recognized at the position (%f, %f, %f).", 
            obj_idx, positions[obj_idx].x(), positions[obj_idx].y(), positions[obj_idx].z());
    
    return obj_idx;
}

void real_bringup::PickAndPlaceUsingCamerasNode::computeObjectApproachAndPickPose()
{
    const Eigen::Vector3f pos = AABB::getPositions(obj_idx);
    const Eigen::Vector3f dim = AABB::getDimensions(obj_idx);
    Eigen::Matrix3f R;
    Eigen::Vector3f RPY, YPR;

    // For approaching from above
    R.col(0) << pos.x(), pos.y(), 0; R.col(0).normalize();
    R.col(1) << pos.y(), -pos.x(), 0; R.col(1).normalize();
    R.col(2) << 0, 0, -1;

    // For approaching by side
    // R.col(0) << 0, 0, -1;
    // R.col(1) << -pos.y(), pos.x(), 0; R.col(1).normalize();
    // R.col(2) << pos.x(), pos.y(), 0; R.col(2).normalize();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rotation matrix: ");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector n: (%f, %f, %f)", R(0,0), R(1,0), R(2,0));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector s: (%f, %f, %f)", R(0,1), R(1,1), R(2,1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector a: (%f, %f, %f)", R(0,2), R(1,2), R(2,2));

    YPR = R.eulerAngles(2, 1, 0);   // R = Rz(yaw) * Ry(pich) * Rx(roll)
    RPY << YPR(2), YPR(1), YPR(0);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RPY: (%f, %f, %f)", RPY(0), RPY(1), RPY(2));
    
    float fi = std::atan2(pos.y(), pos.x());
    float r = pos.head(2).norm();
    if (r < 0.3)
        r += 1.5 * dim.head(2).norm();
    else
        r -= 1.5 * dim.head(2).norm();

    object_approach_pose = {1000 * r * float(cos(fi)), 
                            1000 * r * float(sin(fi)), 
                            1000 * pos.z() + delta_z, 
                            RPY(0), 
                            RPY(1), 
                            RPY(2)};
    object_pick_pose = {1000 * pos.x(), 
                        1000 * pos.y(), 
                        1000 * pos.z() + offset_z, 
                        RPY(0), 
                        RPY(1), 
                        RPY(2)};
}
