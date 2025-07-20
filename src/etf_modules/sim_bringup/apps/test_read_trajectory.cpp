#include "sim_demos/ReadTrajectory.h"
#include <glog/logging.h>

int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	const std::string node_name { "read_trajectory_node" };
	const std::string config_file_path 
	{
		"/sim_bringup/data/read_trajectory_config.yaml"
	};

	rclcpp::init(argc, argv);
	std::shared_ptr<sim_bringup::ReadTrajectory> read_trajectory_node 
	{
		std::make_shared<sim_bringup::ReadTrajectory>(node_name, config_file_path, false)
	};
	
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(read_trajectory_node);
	executor.spin();
	rclcpp::shutdown();

	google::ShutDownCommandLineFlags();
    return 0;
}
