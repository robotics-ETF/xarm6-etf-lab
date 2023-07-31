#include "pick_and_place.h"

int main(int argc, char *argv[])
{
    const std::string node_name = "pick_and_place_node";
	const int period = 4;
    const std::string time_unit = "seconds";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PickAndPlaceNode>(node_name, period, time_unit));
    rclcpp::shutdown();
    return 0;
}