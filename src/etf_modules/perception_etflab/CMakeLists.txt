cmake_minimum_required(VERSION 3.8)
project(perception_etflab)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED common filters segmentation sample_consensus surface)
find_package(pcl_conversions REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_api REQUIRED)
find_package(rpmpl_library REQUIRED)
find_package(nanoflann REQUIRED)
find_package(backward_ros REQUIRED)

include_directories(
	include
	${EIGEN3_INCLUDE_DIR}
	${RPMPL_LIBRARY_INCLUDE_DIRS}
)

# The compiled library code is here
add_subdirectory(src)

# The executable code is here
add_subdirectory(apps)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)

	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	set(ament_cmake_copyright_FOUND TRUE)

	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
