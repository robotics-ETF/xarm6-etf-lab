#include "real_demos/DemoNode2.h"

int main(int argc, char *argv[])
{
    const std::string node_name { "demo2_node" };
	const std::string config_file_path { "/real_bringup/data/demo2_config.yaml" };

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<real_bringup::DemoNode2>(node_name, config_file_path));
    rclcpp::shutdown();

    return 0;
}