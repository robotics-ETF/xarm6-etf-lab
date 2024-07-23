#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


void printEigenMatrix(const Eigen::Matrix4d& matrix) {
    // Convert Eigen::Matrix4d to a string representation
    std::ostringstream oss;
    oss << matrix;

    // Get the string representation of the matrix
    std::string matrixStr = oss.str();

    // Print the matrix using rclcpp logging
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", matrixStr.c_str());
}

void writeMatrixToYaml(const Eigen::Matrix4d& matrix, const std::string& filename) {
    YAML::Emitter out;

    out << YAML::BeginMap;
    out << YAML::Key << "matrix" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < matrix.rows(); ++i) {
        out << YAML::Flow << YAML::BeginSeq;
        for (int j = 0; j < matrix.cols(); ++j) {
            out << matrix(i, j);
        }
        out << YAML::EndSeq;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(filename);
    if (fout.is_open()) {
        fout << out.c_str();
        fout.close();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Matrix written to %s", filename.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open file %s", filename.c_str());
    }
}

Eigen::Matrix4d averageMatrix(const std::vector<Eigen::Matrix4d>& matrices) {
    // Check if the vector is empty
    if (matrices.empty()) {
        throw std::invalid_argument("Cannot compute average of empty vector");
    }

    int numMatrices = matrices.size();
    Eigen::Matrix4d average = Eigen::Matrix4d::Zero();

    // Sum all matrices
    for (const auto& matrix : matrices) {
        average += matrix;
    }

    // Calculate average
    average /= static_cast<double>(numMatrices);

    return average;
}

std::vector<geometry_msgs::msg::Transform> readTransformsFromYAML(const std::string &file_path, const std::string &key) {

    std::vector<geometry_msgs::msg::Transform> transforms;

    // Load the YAML file
    YAML::Node yaml_data = YAML::LoadFile(file_path);

    if (yaml_data[key]) {
        for (const auto &transform_node : yaml_data[key]) {
            geometry_msgs::msg::Transform transform;

            // Extract translation
            transform.translation.x = transform_node["translation"]["x"].as<double>();
            transform.translation.y = transform_node["translation"]["y"].as<double>();
            transform.translation.z = transform_node["translation"]["z"].as<double>();

            // Extract rotation
            transform.rotation.x = transform_node["rotation"]["x"].as<double>();
            transform.rotation.y = transform_node["rotation"]["y"].as<double>();
            transform.rotation.z = transform_node["rotation"]["z"].as<double>();
            transform.rotation.w = transform_node["rotation"]["w"].as<double>();

            // Add the transform to the vector
            transforms.push_back(transform);
        }
    } else {
        std::cerr << "Key " << key << " not found in the YAML file." << std::endl;
    }

    return transforms;
}

Eigen::Matrix4d transformToHomogeneousMatrix(const geometry_msgs::msg::Transform &transform) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

    // Extract the translation
    double x = transform.translation.x;
    double y = transform.translation.y;
    double z = transform.translation.z;

    // Extract the rotation (quaternion)
    double qx = transform.rotation.x;
    double qy = transform.rotation.y;
    double qz = transform.rotation.z;
    double qw = transform.rotation.w;

    // Convert quaternion to rotation matrix
    tf2::Quaternion quaternion(qx, qy, qz, qw);
    tf2::Matrix3x3 rotationMatrix(quaternion);

    // Fill the homogeneous transformation matrix
    matrix(0, 0) = rotationMatrix[0][0];
    matrix(0, 1) = rotationMatrix[0][1];
    matrix(0, 2) = rotationMatrix[0][2];
    matrix(0, 3) = x;
    matrix(1, 0) = rotationMatrix[1][0];
    matrix(1, 1) = rotationMatrix[1][1];
    matrix(1, 2) = rotationMatrix[1][2];
    matrix(1, 3) = y;
    matrix(2, 0) = rotationMatrix[2][0];
    matrix(2, 1) = rotationMatrix[2][1];
    matrix(2, 2) = rotationMatrix[2][2];
    matrix(2, 3) = z;

    return matrix;
}

void processTransform() {

    std::string project_abs_path(__FILE__);
    for (size_t i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
    const std::string calib_pts_file_path = project_abs_path + "/aruco_calibration/aruco_ros/data/calib_points.yaml";
    const std::string camera_coordinates_file_path = project_abs_path + "/aruco_calibration/aruco_ros/data/camera_coordinates.yaml";

    // Read transforms from the YAML file
    auto dir_kin_transforms = readTransformsFromYAML(calib_pts_file_path, "transforms_dir_kin");
    auto camera_transforms = readTransformsFromYAML(calib_pts_file_path, "transforms_camera");

    std::vector<Eigen::Matrix4d> dir_kin_transforms_homo;
    std::vector<Eigen::Matrix4d> camera_transforms_homo;


    for (const auto &transform_node : dir_kin_transforms) {
        dir_kin_transforms_homo.push_back(transformToHomogeneousMatrix(transform_node));
    }

    for (const auto &transform_node : camera_transforms) {
        camera_transforms_homo.push_back(transformToHomogeneousMatrix(transform_node));
    }

    std::vector<Eigen::Matrix4d> camera_cords;

    for (size_t i = 0; i < dir_kin_transforms.size(); i++){
        Eigen::Matrix4d camera_cord = dir_kin_transforms_homo[i] * camera_transforms_homo[i].inverse() ;
        camera_cords.push_back(camera_cord);
    }

    Eigen::Matrix4d camera_cord_avg = averageMatrix(camera_cords);

    writeMatrixToYaml(camera_cord_avg, camera_coordinates_file_path);


}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    processTransform();
    rclcpp::shutdown();
    return 0;
}