#include "real_demos/DemoNode3.h"

int main(int argc, char *argv[])
{
    const std::string node_name { "demo3_node" };
	const std::string config_file_path { "/real_bringup/data/demo3_config.yaml" };

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<real_bringup::DemoNode3>(node_name, config_file_path));
    rclcpp::shutdown();

    return 0;
}