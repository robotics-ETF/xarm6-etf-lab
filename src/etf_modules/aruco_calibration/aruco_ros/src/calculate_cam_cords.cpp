/*

    This .cpp file uses captured homogenous coordinates matrices, averages them,
   transforms them to xyz_RPY format and writes them to camera_coordinates_final

*/

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// used for transformation between camera_left_color_frame and camera_left_link
Eigen::Matrix4d createTransformationMatrix() {
  // Translation vector
  Eigen::Vector3d translation(-0.000113211, 0.0149592, -7.0428e-05);

  // Quaternion (X, Y, Z, W)
  Eigen::Quaterniond quaternion(0.999957, 0.00910854, 0.000204578, 0.0016516);
  quaternion.normalize();  // Ensure the quaternion is normalized

  // Convert quaternion to rotation matrix
  Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

  // Create the 4x4 transformation matrix
  Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
  transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;  // Set rotation
  transformationMatrix.block<3, 1>(0, 3) = translation;     // Set translation

  std::cout << "Matrix optical:\n" << transformationMatrix << "\n\n";

  return transformationMatrix;
}

Eigen::Matrix4d rotatePointOpticalToLink(const Eigen::Matrix4d& point) {
  // Define rotation matrix for -π/2 around the y-axis
  Eigen::Matrix4d rotationY;
  rotationY << 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  // Define rotation matrix for π/2 around the x-axis
  Eigen::Matrix4d rotationX;
  rotationX << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;

  // Apply the rotations to the point
  Eigen::Matrix4d rotatedPoint = point * rotationY;

  rotatedPoint = rotatedPoint * rotationX;

  return rotatedPoint;
}

void printEigenMatrix(const Eigen::Matrix4d& matrix) {
  // Convert Eigen::Matrix4d to a string representation
  std::ostringstream oss;
  oss << matrix;

  // Get the string representation of the matrix
  std::string matrixStr = oss.str();

  // Print the matrix using rclcpp logging
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", matrixStr.c_str());
}

// Function to load matrices from a YAML file
std::vector<Eigen::Matrix4d> loadMatricesFromYAML(const std::string& filename) {
  std::vector<Eigen::Matrix4d> matrices;
  YAML::Node config = YAML::LoadFile(filename);

  for (const auto& entry : config) {
    Eigen::Matrix4d matrix;
    auto matrix_values = entry.second.as<std::vector<std::vector<double>>>();

    // Load each element into the Eigen::Matrix4d
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        matrix(i, j) = matrix_values[i][j];
      }
    }

    matrices.push_back(matrix);
  }

  return matrices;
}

void writeMatrixToYaml(const Eigen::Matrix4d& matrix,
                       const std::string& filename, std::string key_name) {
  // Extract translation
  double x = matrix(0, 3);
  double y = matrix(1, 3);
  double z = matrix(2, 3);

  // Extract rotation matrix (upper 3x3 part)
  Eigen::Matrix3d rotation_matrix = matrix.block<3, 3>(0, 0);

  // Convert rotation matrix to Euler angles (roll, pitch, yaw)
  Eigen::Vector3d euler_angles =
      rotation_matrix.eulerAngles(2, 1, 0);  // ZYX order: yaw, pitch, roll

  double yaw = euler_angles[0];
  double pitch = euler_angles[1];
  double roll = euler_angles[2];

  // Prepare the YAML content
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << key_name << YAML::Value << YAML::Flow << YAML::BeginSeq;
  out << x << y << z << yaw << pitch << roll;
  out << YAML::EndSeq;
  out << YAML::EndMap;

  // Open the file in append mode
  std::ofstream fout(filename, std::ios::out | std::ios::app);
  if (fout.is_open()) {
    fout << out.c_str();  // Write the YAML content
    fout.close();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Transformed matrix appended to %s", filename.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to open file %s for writing", filename.c_str());
  }
}

Eigen::Matrix4d averageMatrix(const std::vector<Eigen::Matrix4d>& matrices) {
  // Check if the vector is empty
  if (matrices.empty()) {
    throw std::invalid_argument("Cannot compute average of empty vector");
  }

  int numMatrices = matrices.size();

  // Variables to store the sum of positions and Euler angles
  Eigen::Vector3d positionSum = Eigen::Vector3d::Zero();
  Eigen::Vector3d eulerSum = Eigen::Vector3d::Zero();

  // Decompose each matrix into translation and rotation (Euler angles)
  for (const auto& matrix : matrices) {
    // Extract position vector (translation part)
    Eigen::Vector3d position = matrix.block<3, 1>(0, 3);

    // Extract rotation matrix and convert to Euler angles (ZYX order)
    Eigen::Matrix3d rotationMatrix = matrix.block<3, 3>(0, 0);
    Eigen::Vector3d eulerAngles =
        rotationMatrix.eulerAngles(2, 1, 0);  // ZYX convention

    // Accumulate positions and Euler angles
    positionSum += position;
    eulerSum += eulerAngles;
  }

  // Calculate the average of positions and Euler angles
  Eigen::Vector3d avgPosition = positionSum / static_cast<double>(numMatrices);
  Eigen::Vector3d avgEulerAngles = eulerSum / static_cast<double>(numMatrices);

  // Convert averaged Euler angles back to a rotation matrix
  Eigen::Matrix3d avgRotationMatrix;
  avgRotationMatrix =
      Eigen::AngleAxisd(avgEulerAngles[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(avgEulerAngles[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(avgEulerAngles[2], Eigen::Vector3d::UnitX());

  // Construct the averaged homogeneous matrix
  Eigen::Matrix4d avgMatrix = Eigen::Matrix4d::Identity();
  avgMatrix.block<3, 3>(0, 0) = avgRotationMatrix;
  avgMatrix.block<3, 1>(0, 3) = avgPosition;

  return avgMatrix;
}

std::vector<geometry_msgs::msg::Transform> readTransformsFromYAML(
    const std::string& file_path, const std::string& key) {
  std::vector<geometry_msgs::msg::Transform> transforms;

  // Load the YAML file
  YAML::Node yaml_data = YAML::LoadFile(file_path);

  if (yaml_data[key]) {
    for (const auto& transform_node : yaml_data[key]) {
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

Eigen::Matrix4d transformToHomogeneousMatrix(
    const geometry_msgs::msg::Transform& transform) {
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

Eigen::VectorXd computeLSE(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  // Check if A and b dimensions are compatible
  if (A.rows() != b.size()) {
    throw std::invalid_argument(
        "Matrix A and vector b have incompatible dimensions.");
  }

  // Compute the Least Squares Estimate
  Eigen::VectorXd x = (A.transpose() * A).ldlt().solve(A.transpose() * b);

  return x;
}

// Function to read data from YAML file and construct A and b
std::pair<Eigen::MatrixXd, Eigen::VectorXd> constructLSEData(
    const std::string& yamlFilePath, const std::string& coord) {
  // Load YAML file
  YAML::Node yamlData = YAML::LoadFile(yamlFilePath);

  // Extract transforms from YAML file
  const auto& transforms_dir_kin = yamlData["transforms_dir_kin"];
  const auto& transforms_camera = yamlData["transforms_camera"];

  // Check data validity
  if (!transforms_dir_kin || !transforms_camera ||
      transforms_dir_kin.size() != transforms_camera.size()) {
    throw std::runtime_error(
        "Invalid or mismatched transforms data in YAML file.");
  }

  // Number of captures
  size_t numCaptures = transforms_dir_kin.size();

  // Initialize A (numCaptures x 4) and b (numCaptures x 1)
  Eigen::MatrixXd A(numCaptures, 4);
  Eigen::VectorXd b(numCaptures);

  // Populate A and b
  for (size_t i = 0; i < numCaptures; ++i) {
    // Extract translation components from transforms_camera
    double X = transforms_camera[i]["translation"]["x"].as<double>();
    double Y = transforms_camera[i]["translation"]["y"].as<double>();
    double Z = transforms_camera[i]["translation"]["z"].as<double>();

    Eigen::VectorXd tmp_row(4);
    tmp_row << X, Y, Z, 1.0;

    Eigen::Matrix4d color_to_link_transform = createTransformationMatrix();

    tmp_row = color_to_link_transform * tmp_row;

    // Fill A matrix row
    A.row(i) = tmp_row.transpose();

    // Extract the specified coordinate for b from transforms_dir_kin
    if (coord == "x") {
      b(i) = transforms_dir_kin[i]["translation"]["x"].as<double>();
    } else if (coord == "y") {
      b(i) = transforms_dir_kin[i]["translation"]["y"].as<double>();
    } else if (coord == "z") {
      b(i) = transforms_dir_kin[i]["translation"]["z"].as<double>();
    } else {
      throw std::invalid_argument("Invalid coordinate specified for b: " +
                                  coord);
    }
  }

  return {A, b};
}

Eigen::Matrix4d createHomogeneousMatrix(const Eigen::VectorXd& x,
                                        const Eigen::VectorXd& y,
                                        const Eigen::VectorXd& z) {
  // Ensure that input vectors have 4 components
  assert(x.size() == 4 && y.size() == 4 && z.size() == 4 &&
         "Input vectors must have 4 components each.");

  Eigen::Matrix4d matrix;

  // Assign rows from the vectors
  matrix.row(0) = x.transpose();
  matrix.row(1) = y.transpose();
  matrix.row(2) = z.transpose();

  // Add the last row for homogeneous coordinates
  matrix.row(3) << 0, 0, 0, 1;

  return matrix;
}

void processTransform(std::string camera_side) {
  std::string project_abs_path(__FILE__);
  for (size_t i = 0; i < 4; i++)
    project_abs_path =
        project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

  const std::string calib_pts_file_path =
      project_abs_path + "/aruco_calibration/aruco_ros/data/" + camera_side +
      "_camera/calib_points.yaml";
  const std::string camera_coordinates_file_path_final =
      project_abs_path + "/aruco_calibration/aruco_ros/data/" + camera_side +
      "_camera/camera_coordinates_final.yaml";
  const std::string camera_coordinates_file_path_all =
      project_abs_path + "/aruco_calibration/aruco_ros/data/" + camera_side +
      "_camera/camera_coordinates_all.yaml";

  /*###############
 CALCULATION USING AVERAGING
 #################*/

  auto dir_kin_transforms =
      readTransformsFromYAML(calib_pts_file_path, "transforms_dir_kin");
  auto camera_transforms =
      readTransformsFromYAML(calib_pts_file_path, "transforms_camera");

  std::vector<Eigen::Matrix4d> dir_kin_transforms_homo;
  std::vector<Eigen::Matrix4d> camera_transforms_homo;

  for (const auto& transform_node : dir_kin_transforms) {
    dir_kin_transforms_homo.push_back(
        transformToHomogeneousMatrix(transform_node));
  }

  for (const auto& transform_node : camera_transforms) {
    camera_transforms_homo.push_back(
        transformToHomogeneousMatrix(transform_node));
  }

  Eigen::Matrix4d color_to_link_transform = createTransformationMatrix();

  std::vector<Eigen::Matrix4d> camera_cords;

  for (size_t i = 0; i < dir_kin_transforms.size(); i++) {
    Eigen::Matrix4d camera_cord = dir_kin_transforms_homo[i] *
                                  camera_transforms_homo[i].inverse() *
                                  color_to_link_transform.inverse();
    writeMatrixToYaml(camera_cord, camera_coordinates_file_path_all,
                      "matrix" + std::to_string(i));
    camera_cords.push_back(camera_cord);
  }

  Eigen::Matrix4d camera_cord_avg = averageMatrix(camera_cords);

  camera_cord_avg = rotatePointOpticalToLink(camera_cord_avg);

  writeMatrixToYaml(camera_cord_avg, camera_coordinates_file_path_final,
                    "xyz_YPR");

  /*###############
  CALCULATION USING LSE
  #################*/

  try {
    std::string yamlFilePath =
        calib_pts_file_path;  // Replace with your YAML file path
    std::string coord =
        "y";  // Coordinate to use for b (e.g., "x", "y", or "z")

    auto [Ax, bx] = constructLSEData(yamlFilePath, "x");
    auto [Ay, by] = constructLSEData(yamlFilePath, "y");
    auto [Az, bz] = constructLSEData(yamlFilePath, "z");

    Eigen::VectorXd x = computeLSE(Ax, bx);
    Eigen::VectorXd y = computeLSE(Ay, by);
    Eigen::VectorXd z = computeLSE(Az, bz);

    Eigen::Matrix4d camera_in_world = createHomogeneousMatrix(x, y, z);

    camera_in_world = rotatePointOpticalToLink(camera_in_world);

    writeMatrixToYaml(camera_in_world, camera_coordinates_file_path_final,
                      "xyz_YPR_lse");

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("calibration_calc_node");

  node->declare_parameter<std::string>("camera_side", "left");

  std::string camera_side =
      node->get_parameter("camera_side").get_value<std::string>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n \n Camera side: %s \n \n",
              camera_side.c_str());

  processTransform(camera_side);

  rclcpp::shutdown();
  return 0;
}