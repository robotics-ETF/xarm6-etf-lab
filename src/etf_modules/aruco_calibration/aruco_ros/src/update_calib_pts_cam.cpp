/*
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
*/
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <xArm6.h>
#include <RealVectorSpace.h>
#include <yaml-cpp/yaml.h>

#include <sstream>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2_kdl/tf2_kdl.h"

class TransformUpdateNode : public rclcpp::Node
{
public:
    TransformUpdateNode()
        : Node("transform_update_node")
    {
        subscription_camera = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "aruco_single/transform", 10,
            std::bind(&TransformUpdateNode::camera_callback, this, std::placeholders::_1));
    }

private:
    void camera_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)

    {
        // Extract translation and rotation


        YAML::Node transform_data;
        transform_data["translation"]["x"] = msg->transform.translation.x;
        transform_data["translation"]["y"] = msg->transform.translation.y;
        transform_data["translation"]["z"] = msg->transform.translation.z;
        transform_data["rotation"]["x"] = msg->transform.rotation.x;
        transform_data["rotation"]["y"] = msg->transform.rotation.y;
        transform_data["rotation"]["z"] = msg->transform.rotation.z;
        transform_data["rotation"]["w"] = msg->transform.rotation.w;

        std::string project_abs_path(__FILE__);
        for (size_t i = 0; i < 4; i++)
		  project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
        
        const std::string config_file_path = "/aruco_calibration/aruco_ros/data/calib_points.yaml";
        
        YAML::Node yaml_data;
        std::string yaml_file_path = project_abs_path + config_file_path;
        // Load existing data from the YAML file
        
        try {
            
            if (std::ifstream yaml_file(yaml_file_path); yaml_file.is_open()) {
            yaml_data = YAML::Load(yaml_file);
            }
            
            // Ensure 'transforms' key exists and is a sequence
            if (!yaml_data["transforms_camera"]) {
                yaml_data["transforms_camera"] = YAML::Node(YAML::NodeType::Sequence);
            }

            // Append the new transform data to the list
            yaml_data["transforms_camera"].push_back(transform_data);

            // Write the updated data back to the YAML file
            std::ofstream yaml_out(yaml_file_path);
            yaml_out << yaml_data;

            RCLCPP_INFO(this->get_logger(), "YAML file updated with new transform data.");

            // Shutdown the node after the first update
            rclcpp::shutdown();
            
        }
        catch (...) {
            std::cout << "Problem accessing YAML file.\n";
        }

    }

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_camera;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformUpdateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}