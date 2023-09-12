#include "environment/Octomap.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
	const std::string config_file_path = "";
    sim_bringup::Octomap octomap(config_file_path);    
    int num = 0;
    while (num++ < 10)
        octomap.read();

    rclcpp::shutdown();
    return 0;
}