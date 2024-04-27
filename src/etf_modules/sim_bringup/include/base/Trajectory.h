//
// Created by nermin on 28.08.23.
//

#ifndef SIM_BRINGUP_TRAJECTORY_H
#define SIM_BRINGUP_TRAJECTORY_H

#include "base/Robot.h"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <yaml-cpp/yaml.h>

#include <Spline5.h>

namespace sim_bringup
{
    class Trajectory : public sim_bringup::Robot
    {
    public:
        Trajectory(const std::string &config_file_path);
        ~Trajectory() {}
        
        void addPoint(float time_instance, const Eigen::VectorXf &position);
        void addPoint(float time_instance, const Eigen::VectorXf &position, const Eigen::VectorXf &velocity);
        void addPoint(float time_instance, const Eigen::VectorXf &position, const Eigen::VectorXf &velocity, 
                      const Eigen::VectorXf &acceleration);
        void addPoints(std::shared_ptr<planning::trajectory::Spline> spline, float t_offset, float t_final);
        
        void addPath(const std::vector<std::shared_ptr<base::State>> &path, const std::vector<float> &time_instances);
        void addPath(const std::vector<Eigen::VectorXf> &path, const std::vector<float> &time_instances);
        void addPath(const std::vector<std::shared_ptr<base::State>> &path);
        void addPath(const std::vector<std::shared_ptr<base::State>> &path, bool must_visit);

        void publish(bool print = false);
        void clear();
        inline size_t getNumPoints() const { return msg.points.size(); }
        
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;

    private:
        trajectory_msgs::msg::JointTrajectory msg;
        float trajectory_max_time_step;
    };
}

#endif // SIM_BRINGUP_TRAJECTORY_H