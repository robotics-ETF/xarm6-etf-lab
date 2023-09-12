#include "base/Robot.h"
#include "base/Trajectory.h"

#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace sim_bringup
{
    class BaseNode : public rclcpp::Node, 
                     public sim_bringup::Robot, 
                     public sim_bringup::Trajectory
    {
    public:
        BaseNode(const std::string node_name, const std::string config_file_path);
        virtual void baseCallback() = 0;

        rclcpp::TimerBase::SharedPtr timer;
        int period;     // Period of basic callback function in [ms]
        std::string project_abs_path;

    };
}