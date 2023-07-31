#include "move_xarm6.h"


int main(int argc, char *argv[])
{
	const std::string node_name = "move_xarm6_node";
	const int period = 5;
    const std::string time_unit = "seconds";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveXArm6Node>(node_name, period, time_unit));
    rclcpp::shutdown();
    return 0;
}