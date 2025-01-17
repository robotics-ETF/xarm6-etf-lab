#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

// Function to calculate mean
double calculateMean(const std::vector<double>& data) {
  return std::accumulate(data.begin(), data.end(), 0.0) / data.size();
}

// Function to calculate standard deviation
double calculateStdDev(const std::vector<double>& data, double mean) {
  double variance = 0.0;
  for (double value : data) {
    variance += (value - mean) * (value - mean);
  }
  variance /= data.size();
  return std::sqrt(variance);
}

int main() {
  try {
    // Load YAML file
    YAML::Node config = YAML::LoadFile(
        "/home/roboticsetf/xarm6-etf-lab/src/etf_modules/aruco_calibration/"
        "aruco_ros/data/left_camera/camera_coordinates_all.yaml");

    // Container to store data
    std::vector<std::vector<double>> allData;

    // Extract data from YAML
    for (const auto& matrix : config) {
      std::vector<double> values = matrix.second.as<std::vector<double>>();
      allData.push_back(values);
    }

    // Calculate statistics for each column
    size_t numColumns = allData[0].size();
    for (size_t col = 0; col < numColumns; ++col) {
      std::vector<double> columnData;
      for (const auto& row : allData) {
        columnData.push_back(row[col]);
      }

      double mean = calculateMean(columnData);
      double stddev = calculateStdDev(columnData, mean);

      std::cout << "Column " << col + 1 << ":\n";
      std::cout << "  Mean: " << mean << "\n";
      std::cout << "  Standard Deviation: " << stddev << "\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}