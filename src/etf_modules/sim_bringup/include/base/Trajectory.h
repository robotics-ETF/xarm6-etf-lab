//
// Created by nermin on 28.08.23.
//

#ifndef SIM_BRINGUP_TRAJECTORY_H
#define SIM_BRINGUP_TRAJECTORY_H

#include "base/Robot.h"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <yaml-cpp/yaml.h>

#include <AbstractTrajectory.h>

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
        void addPoints(const std::shared_ptr<planning::trajectory::AbstractTrajectory> trajectory, 
                       float t_begin, float t_end, float t_offset = 0);
        
        void addTrajectory(const std::vector<std::shared_ptr<base::State>> &path, const std::vector<float> &time_instances);
        void addTrajectory(const std::vector<Eigen::VectorXf> &path, const std::vector<float> &time_instances);
        void addTrajectory(const std::shared_ptr<planning::trajectory::AbstractTrajectory> trajectory);

        void publish(bool print = false);
        void clear();
        inline size_t getNumPoints() const { return msg.points.size(); }
        inline float getTrajectoryMaxTimeStep() const { return trajectory_max_time_step; }
        
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;
        bool ready;

    private:
        trajectory_msgs::msg::JointTrajectory msg;
        float trajectory_max_time_step;
    };
}

#endif // SIM_BRINGUP_TRAJECTORY_H