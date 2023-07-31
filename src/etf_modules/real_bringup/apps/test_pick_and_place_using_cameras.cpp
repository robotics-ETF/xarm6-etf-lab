#include "pick_and_place_using_cameras.h"

int main(int argc, char *argv[])
{
    const std::string node_name = "pick_and_place_using_cameras_node";
	const int period = 5;
    const std::string time_unit = "seconds";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PickAndPlaceUsingCamerasNode>(node_name, period, time_unit));
    rclcpp::shutdown();
    return 0;
}