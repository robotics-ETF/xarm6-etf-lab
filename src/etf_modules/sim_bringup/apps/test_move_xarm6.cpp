#include "base/MoveXArm6Node.h"

int main(int argc, char *argv[])
{
	const std::string node_name = "move_xarm6_node";
	const std::string config_file_path = "/sim_bringup/data/move_xarm6_config.yaml";
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sim_bringup::MoveXArm6Node>(node_name, config_file_path));
    rclcpp::shutdown();
    return 0;
}