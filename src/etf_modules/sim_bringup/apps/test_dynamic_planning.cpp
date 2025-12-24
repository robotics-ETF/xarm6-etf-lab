#include "sim_demos/DynamicPlanningNode.h"
#include "sim_demos/DynamicPlanningNode2.h"
#include <glog/logging.h>

int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	const std::string node_name { "dynamic_planning_node" };
	const std::string config_file_path 
	{
		"/sim_bringup/data/dynamic_planning_config1.yaml"
		// "/sim_bringup/data/dynamic_planning_config2.yaml"
	};

	rclcpp::init(argc, argv);
	std::shared_ptr<sim_bringup::DynamicPlanningNode> dynamic_planning_node 
	{
		std::make_shared<sim_bringup::DynamicPlanningNode>(node_name, config_file_path, true)
		// std::make_shared<sim_bringup::DynamicPlanningNode>(node_name, config_file_path, true, "_test.log")	// NOTE: Recordings of a measured trajectory will not work in the simulation!
		// std::make_shared<sim_bringup::DynamicPlanningNode2>(node_name, config_file_path, true)
	};
	
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(dynamic_planning_node);
	executor.spin();
	rclcpp::shutdown();

	google::ShutDownCommandLineFlags();
    return 0;
}
