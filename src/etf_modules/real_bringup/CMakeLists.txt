cmake_minimum_required(VERSION 3.8)
project(real_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_api REQUIRED)
find_package(fcl 0.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rpmpl_library REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${RPMPL_LIBRARY_INCLUDE_DIRS}
)

add_executable(test_move_xarm6 src/test_move_xarm6.cpp)
target_compile_features(test_move_xarm6 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_move_xarm6 PUBLIC 
  rclcpp 
  rclcpp_action 
  trajectory_msgs 
  control_msgs 
  xarm_msgs 
  xarm_api
)

add_executable(test_bottle_and_glass src/test_bottle_and_glass.cpp)
target_compile_features(test_bottle_and_glass PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_bottle_and_glass PUBLIC 
  rclcpp 
  xarm_msgs 
  xarm_api
)

add_executable(test_pick_and_place src/test_pick_and_place.cpp)
target_compile_features(test_pick_and_place PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_pick_and_place PUBLIC 
  rclcpp 
  xarm_msgs 
  xarm_api
)

add_executable(test_pick_and_place_using_cameras src/test_pick_and_place_using_cameras.cpp)
target_compile_features(test_pick_and_place_using_cameras PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_pick_and_place_using_cameras PUBLIC 
  rclcpp 
  xarm_msgs 
  xarm_api 
  sensor_msgs 
  Eigen3
  PCL
  pcl_conversions
)

add_executable(test_planners src/test_planners.cpp)
target_compile_features(test_planners PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_planners PUBLIC ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})
ament_target_dependencies(test_planners PUBLIC 
  rclcpp 
  trajectory_msgs
  visualization_msgs 
  control_msgs 
  sensor_msgs
  fcl 
  Eigen3 
  PCL
  pcl_conversions 
)

install(TARGETS 
  test_move_xarm6
  test_bottle_and_glass
  test_pick_and_place
  test_pick_and_place_using_cameras
  test_planners
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  scripts/test_move_xarm6.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
