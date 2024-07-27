#include "sim_demos/RealTimePlanningNode.h"

#include <xarm_api/xarm_ros_client.h>

namespace real_bringup
{
    class RealTimePlanningNode : public sim_bringup::RealTimePlanningNode
    {
    public:
        RealTimePlanningNode(const std::string &node_name, const std::string &config_file_path, bool loop_, 
                             const std::string &output_file_name = "");

    protected:
        void computeTrajectory() override;
        void publishingTrajectoryCallback();

        rclcpp::TimerBase::SharedPtr publishing_trajectory_timer;
        std::shared_ptr<rclcpp::Node> xarm_client_node;
        xarm_api::XArmROSClient xarm_client;
    };
}
