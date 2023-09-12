#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

#include <RealVectorSpaceState.h>

namespace sim_bringup
{
    class Robot
    {
    public:
        Robot(const std::string config_file_path);
        
        inline const std::shared_ptr<base::State> getJointsState() { return joints_state; }
        inline const std::shared_ptr<base::State> getHomeJointsState() { return home_joints_state; }
        inline const float getMaxLinVel() { return max_lin_vel; }
        inline const float getMaxLinAcc() { return max_lin_acc; }
        inline const float getMaxAngVel() { return max_ang_vel; }
        inline const float getMaxAngAcc() { return max_ang_acc; }
        inline void setMaxLinVel(float max_lin_vel_) { max_lin_vel = max_lin_vel_; }
        inline void setMaxLinAcc(float max_lin_acc_) { max_lin_acc = max_lin_acc_; }
        inline void setMaxAngVel(float max_ang_vel_) { max_ang_vel = max_ang_vel_; }
        inline void setMaxAngAcc(float max_ang_acc_) { max_ang_acc = max_ang_acc_; }
        inline void setHomeJointsState(const Eigen::VectorXf &home_joints_coord) 
            { home_joints_state = std::make_shared<base::RealVectorSpaceState>(home_joints_coord); }

        void jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
        bool isReady();
        bool isReached(std::shared_ptr<base::State> q, float tol = 0.01);
        
        rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joints_state_subscription;

    private:
        std::shared_ptr<base::State> joints_state;
        std::shared_ptr<base::State> home_joints_state;
        float max_lin_vel;  // in [mm/s]
        float max_lin_acc;  // in [mm/s²]
        float max_ang_vel;  // in [rad/s]
        float max_ang_acc;  // in [rad/s²]
    };
}