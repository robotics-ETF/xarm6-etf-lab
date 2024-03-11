#include "demos/MoveXArm6Node.h"
#include <glog/logging.h>

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";
    
	const std::string node_name = "move_xarm6_node";
	const std::string config_file_path = "/sim_bringup/data/move_xarm6_config.yaml";
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sim_bringup::MoveXArm6Node>(node_name, config_file_path));
    rclcpp::shutdown();
    return 0;
}