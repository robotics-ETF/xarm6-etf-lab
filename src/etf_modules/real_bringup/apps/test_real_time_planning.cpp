#include "real_demos/RealTimePlanningNode.h"
#include <glog/logging.h>

int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	const std::string node_name { "real_time_planning_node" };
	const std::string config_file_path 
	{
		"/real_bringup/data/real_time_planning_config1.yaml"
		// "/real_bringup/data/real_time_planning_config2.yaml"
	};

	rclcpp::init(argc, argv);
	std::shared_ptr<real_bringup::RealTimePlanningNode> real_time_planning_node
	{
		std::make_shared<real_bringup::RealTimePlanningNode>(node_name, config_file_path, true)
		// std::make_shared<real_bringup::RealTimePlanningNode>(node_name, config_file_path, true, "_test.log")
	};
	
	rclcpp::init(argc, argv);
	rclcpp::spin(real_time_planning_node);
	rclcpp::shutdown();

	// For some reason, the following does not work:
	// rclcpp::executors::MultiThreadedExecutor executor;
	// executor.add_node(real_time_planning_node);
	// executor.spin();
	// rclcpp::shutdown();

	google::ShutDownCommandLineFlags();
    return 0;
}
