#include "planning.h"

int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	const std::string scenario_file_path = "/real_bringup/data/scenario_etf_lab.yaml";
	const std::string node_name = "planning_node";
	const int period = 1;
    const std::string time_unit = "seconds";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanningNode>(scenario_file_path, node_name, period, time_unit));
    rclcpp::shutdown();
	google::ShutDownCommandLineFlags();
    return 0;
}