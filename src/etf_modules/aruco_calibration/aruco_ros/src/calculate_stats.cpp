#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

struct Vector3 {
  double x, y, z;
};

struct Quaternion {
  double x, y, z, w;
};

struct Transform {
  Vector3 translation;
  Quaternion rotation;
};

// Function to calculate mean of a vector of doubles
double calculateMean(const std::vector<double>& data) {
  double sum = 0.0;
  for (const auto& value : data) {
    sum += value;
  }
  return sum / data.size();
}

// Function to calculate standard deviation of a vector of doubles
double calculateStdDev(const std::vector<double>& data, double mean) {
  double variance = 0.0;
  for (const auto& value : data) {
    variance += (value - mean) * (value - mean);
  }
  return std::sqrt(variance / data.size());
}

int main() {
  std::string filename =
      "/home/roboticsetf/xarm6-etf-lab/src/etf_modules/aruco_calibration//"
      "aruco_ros/data/calib_points.yaml";  // Update with the path to your
                                           // YAML file

  YAML::Node config;
  try {
    config = YAML::LoadFile(filename);
  } catch (const YAML::BadFile& e) {
    std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    return 1;
  }

  std::vector<Transform> transforms_dir_kin;
  std::vector<Transform> transforms_camera;

  // Parse transforms_dir_kin
  for (const auto& node : config["transforms_dir_kin"]) {
    Transform t;
    t.translation.x = node["translation"]["x"].as<double>();
    t.translation.y = node["translation"]["y"].as<double>();
    t.translation.z = node["translation"]["z"].as<double>();
    t.rotation.x = node["rotation"]["x"].as<double>();
    t.rotation.y = node["rotation"]["y"].as<double>();
    t.rotation.z = node["rotation"]["z"].as<double>();
    t.rotation.w = node["rotation"]["w"].as<double>();
    transforms_dir_kin.push_back(t);
  }

  // Parse transforms_camera
  for (const auto& node : config["transforms_camera"]) {
    Transform t;
    t.translation.x = node["translation"]["x"].as<double>();
    t.translation.y = node["translation"]["y"].as<double>();
    t.translation.z = node["translation"]["z"].as<double>();
    t.rotation.x = node["rotation"]["x"].as<double>();
    t.rotation.y = node["rotation"]["y"].as<double>();
    t.rotation.z = node["rotation"]["z"].as<double>();
    t.rotation.w = node["rotation"]["w"].as<double>();
    transforms_camera.push_back(t);
  }

  std::vector<std::string> categories = {"transforms_dir_kin",
                                         "transforms_camera"};
  std::vector<std::vector<Transform>> all_transforms = {transforms_dir_kin,
                                                        transforms_camera};

  for (size_t i = 0; i < categories.size(); ++i) {
    const auto& transforms = all_transforms[i];
    std::cout << "\nStatistics for " << categories[i] << ":\n";

    std::vector<double> x_vals, y_vals, z_vals;
    std::vector<double> rot_x_vals, rot_y_vals, rot_z_vals, rot_w_vals;

    for (const auto& t : transforms) {
      x_vals.push_back(t.translation.x);
      y_vals.push_back(t.translation.y);
      z_vals.push_back(t.translation.z);

      rot_x_vals.push_back(t.rotation.x);
      rot_y_vals.push_back(t.rotation.y);
      rot_z_vals.push_back(t.rotation.z);
      rot_w_vals.push_back(t.rotation.w);
    }

    // Calculate statistics for translation
    double x_mean = calculateMean(x_vals);
    double y_mean = calculateMean(y_vals);
    double z_mean = calculateMean(z_vals);

    double x_stddev = calculateStdDev(x_vals, x_mean);
    double y_stddev = calculateStdDev(y_vals, y_mean);
    double z_stddev = calculateStdDev(z_vals, z_mean);

    std::cout << "Translation:\n";
    std::cout << "  Mean: (" << x_mean << ", " << y_mean << ", " << z_mean
              << ")\n";
    std::cout << "  Std Dev: (" << x_stddev << ", " << y_stddev << ", "
              << z_stddev << ")\n";

    // Calculate statistics for rotation
    double rot_x_mean = calculateMean(rot_x_vals);
    double rot_y_mean = calculateMean(rot_y_vals);
    double rot_z_mean = calculateMean(rot_z_vals);
    double rot_w_mean = calculateMean(rot_w_vals);

    double rot_x_stddev = calculateStdDev(rot_x_vals, rot_x_mean);
    double rot_y_stddev = calculateStdDev(rot_y_vals, rot_y_mean);
    double rot_z_stddev = calculateStdDev(rot_z_vals, rot_z_mean);
    double rot_w_stddev = calculateStdDev(rot_w_vals, rot_w_mean);

    std::cout << "Rotation:\n";
    std::cout << "  Mean: (" << rot_x_mean << ", " << rot_y_mean << ", "
              << rot_z_mean << ", " << rot_w_mean << ")\n";
    std::cout << "  Std Dev: (" << rot_x_stddev << ", " << rot_y_stddev << ", "
              << rot_z_stddev << ", " << rot_w_stddev << ")\n";
  }

  return 0;
}
