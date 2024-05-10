#include "real_demos/RealTimePlanningNode.h"
#include <glog/logging.h>

int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	const std::string node_name = "real_time_planning_node";
	const std::string config_file_path0 = "/real_bringup/data/real_time_planning_config0.yaml";
	const std::string config_file_path1 = "/real_bringup/data/real_time_planning_config1.yaml";
	const std::string config_file_path2 = "/real_bringup/data/real_time_planning_config2.yaml";
	size_t num { 1 };

	while (true)
	{
		rclcpp::init(argc, argv);
		rclcpp::spin(std::make_shared<real_bringup::RealTimePlanningNode>(node_name, config_file_path1, "_test" + std::to_string(num) + ".log"));
		rclcpp::shutdown();
		rclcpp::sleep_for(std::chrono::milliseconds(100));

		rclcpp::init(argc, argv);
		rclcpp::spin(std::make_shared<real_bringup::RealTimePlanningNode>(node_name, config_file_path2, "_test" + std::to_string(num) + ".log"));
		rclcpp::shutdown();
		rclcpp::sleep_for(std::chrono::milliseconds(100));

		num++;
	}

	google::ShutDownCommandLineFlags();
    return 0;
}
