#include "planning/PlanningNode.h"

int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	const std::string node_name = "planning_node";
	const std::string config_file_path = "/real_bringup/data/planning_config.yaml";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<real_bringup::PlanningNode>(node_name, config_file_path));
    rclcpp::shutdown();
	google::ShutDownCommandLineFlags();
    return 0;
}