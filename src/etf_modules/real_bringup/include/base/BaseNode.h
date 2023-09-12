#include "base/Robot.h"
#include "base/Trajectory.h"

#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace real_bringup
{
    class BaseNode : public rclcpp::Node, 
                     public real_bringup::Robot, 
                     public real_bringup::Trajectory
    {
    public:
        BaseNode(const std::string node_name, const std::string config_file_path);
        virtual void baseCallback() = 0;

        rclcpp::TimerBase::SharedPtr timer;
        int period;     // Period of basic callback function in [ms]
        std::string project_abs_path;

    };
}