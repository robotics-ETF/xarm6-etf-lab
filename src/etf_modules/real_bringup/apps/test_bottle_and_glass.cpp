#include "bottle_and_glass.h"

int main(int argc, char *argv[])
{
	const std::string node_name = "bottle_and_glass_node";
	const int period = 4;
    const std::string time_unit = "seconds";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BottleAndGlassNode>(node_name, period, time_unit));
    rclcpp::shutdown();
    return 0;
}