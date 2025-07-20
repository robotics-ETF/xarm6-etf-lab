#include "sim_demos/RealTimePlanningNode.h"
#include "sim_demos/RealTimePlanningNode2.h"
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
		"/sim_bringup/data/real_time_planning_config1.yaml"
		// "/sim_bringup/data/real_time_planning_config2.yaml"
	};

	rclcpp::init(argc, argv);
	std::shared_ptr<sim_bringup::RealTimePlanningNode> real_time_planning_node 
	{
		std::make_shared<sim_bringup::RealTimePlanningNode>(node_name, config_file_path, true)
		// std::make_shared<sim_bringup::RealTimePlanningNode>(node_name, config_file_path, true, "_test.log")
		// std::make_shared<sim_bringup::RealTimePlanningNode2>(node_name, config_file_path, true)
	};
	
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(real_time_planning_node);
	executor.spin();
	rclcpp::shutdown();

	google::ShutDownCommandLineFlags();
    return 0;
}
