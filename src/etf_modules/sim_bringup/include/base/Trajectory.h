//
// Created by nermin on 28.08.23.
//

#ifndef SIM_BRINGUP_TRAJECTORY_H
#define SIM_BRINGUP_TRAJECTORY_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <yaml-cpp/yaml.h>

#include <State.h>

namespace sim_bringup
{
    class Trajectory
    {
    public:
        Trajectory(const std::string &config_file_path);
        ~Trajectory() {}
        
        void addPoint(float time_instance, const Eigen::VectorXf &position);
        void addPoint(float time_instance, const Eigen::VectorXf &position, const Eigen::VectorXf &velocity);
        void addPoint(float time_instance, const Eigen::VectorXf &position, const Eigen::VectorXf &velocity, 
                      const Eigen::VectorXf &acceleration);
        
        void addPath(const std::vector<std::shared_ptr<base::State>> &path, const std::vector<float> &time_instances);
        void addPath(const std::vector<Eigen::VectorXf> &path, const std::vector<float> &time_instances);
        void addPath(const std::vector<std::shared_ptr<base::State>> &path);

        void publish();
        void clear();
        
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;

    private:
        trajectory_msgs::msg::JointTrajectory msg;
    };
}

#endif // SIM_BRINGUP_TRAJECTORY_H