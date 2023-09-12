# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.

set(INCLUDE_DIRS 
    ${PROJECT_SOURCE_DIR}/include
)

# Optionally glob, but only for CMake 3.12 or later:
file(GLOB_RECURSE HEADER_LIST CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/include/*.h")

file(GLOB_RECURSE SRC_LIST CONFIGURE_DEPENDS "*.cpp")

# message(${SRC_LIST})
# Make an automatic library - will be static or dynamic based on user setting
add_library(perception_etflab_library SHARED ${SRC_LIST} ${HEADER_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(perception_etflab_library PUBLIC ${INCLUDE_DIRS})

# All users of this library will need at least C++17
target_compile_features(perception_etflab_library PUBLIC cxx_std_17)

target_link_libraries(perception_etflab_library PUBLIC ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})

ament_target_dependencies(perception_etflab_library PUBLIC 
  rclcpp
  sensor_msgs
  geometry_msgs
  control_msgs
  visualization_msgs
  pcl_ros
  PCL
  pcl_conversions
  tf2_sensor_msgs
  tf2_eigen
  Eigen3
  xarm_msgs
  xarm_api
)

# IDEs should put the headers in a nice place
source_group(
  TREE "${PROJECT_SOURCE_DIR}/include"
  PREFIX "Header Files"
  FILES ${HEADER_LIST})