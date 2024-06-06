//
// Created by nermin on 28.08.23.
//

#ifndef SIM_BRINGUP_TRAJECTORY_H
#define SIM_BRINGUP_TRAJECTORY_H

#include "base/Robot.h"

#include <visualization_msgs/msg/marker_array.hpp>
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
        
        void addPath(const std::vector<std::shared_ptr<base::State>> &path, const std::vector<float> &time_instances);
        void addPath(const std::vector<Eigen::VectorXf> &path, const std::vector<float> &time_instances);
        void addPath(const std::vector<std::shared_ptr<base::State>> &path);
        
        void publish(float time_delay = 0);
        void publish_markers();
        void clear();
        void clear_markers();
        
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;

        void addMarker(KDL::Vector coordinates, int instance);
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;

    private:
        void preprocessPath(const std::vector<std::shared_ptr<base::State>> &path, std::vector<Eigen::VectorXf> &new_path);

        trajectory_msgs::msg::JointTrajectory msg;
        
        float max_edge_length;
        float trajectory_max_time_step;

        float marker_size;

        visualization_msgs::msg::MarkerArray markers;

    };
}

#endif // SIM_BRINGUP_TRAJECTORY_H