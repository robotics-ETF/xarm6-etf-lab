cmake_minimum_required(VERSION 3.8)
project(aruco_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

set(THIS_PACKAGE_INCLUDE_DEPENDS
    OpenCV
    cv_bridge
    geometry_msgs
    image_transport
    rclcpp
    rclpy
    tf2
    tf2_ros
    tf2_geometry_msgs
    aruco
    aruco_msgs
    sensor_msgs
    visualization_msgs
    control_msgs
    tf2_kdl
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(fcl 0.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nanoflann REQUIRED)
find_package(rpmpl_library REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${RPMPL_LIBRARY_INCLUDE_DIRS}
)

#generate_dynamic_reconfigure_options(
#  cfg/ArucoThreshold.cfg
#)

add_library(aruco_ros_utils SHARED src/aruco_ros_utils.cpp)
target_include_directories(aruco_ros_utils
  PUBLIC
  include)

target_include_directories(aruco_ros_utils
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(aruco_ros_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(aruco_ros_utils ${OpenCV_LIBRARIES})

add_executable(single src/simple_single.cpp
                      src/aruco_ros_utils.cpp)

target_include_directories(single
  PUBLIC
  include)

target_include_directories(single
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(single ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(single ${OpenCV_LIBRARIES})

add_executable(double src/simple_double.cpp
                      src/aruco_ros_utils.cpp)
target_include_directories(double
  PUBLIC
  include)

target_include_directories(double
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(double ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(double ${OpenCV_LIBRARIES})

add_executable(marker_publisher src/marker_publish.cpp
                                src/aruco_ros_utils.cpp)
target_include_directories(marker_publisher
  PUBLIC
  include)

target_include_directories(marker_publisher
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(marker_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(marker_publisher ${OpenCV_LIBRARIES})

# update_points_dir_kin

add_executable(update_calib_points_dir_kin src/update_calib_points_dir_kin.cpp)
target_include_directories(update_calib_points_dir_kin
  PUBLIC
  include)

target_include_directories(update_calib_points_dir_kin
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(update_calib_points_dir_kin ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(update_calib_points_dir_kin ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})

# update_calib_pts_cam

add_executable(update_calib_pts_cam src/update_calib_pts_cam.cpp)
target_include_directories(update_calib_pts_cam
  PUBLIC
  include)

target_include_directories(update_calib_pts_cam
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(update_calib_pts_cam ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(update_calib_pts_cam ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})


# calculate camera coordinates

add_executable(calculate_cam_cords src/calculate_cam_cords.cpp)
target_include_directories(calculate_cam_cords
  PUBLIC
  include)

target_include_directories(calculate_cam_cords
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(calculate_cam_cords ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(calculate_cam_cords ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})

# camera calibration

add_executable(camera_calibration src/CameraCalibrationNode.cpp)

target_include_directories(camera_calibration
  PUBLIC
  include)

target_include_directories(camera_calibration
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(camera_calibration ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(camera_calibration ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()
endif()

# for tf2 broadcaster

add_executable(marker_tf2_broadcaster src/marker_tf2_broadcaster.cpp)

target_include_directories(marker_tf2_broadcaster
  PUBLIC
  include)

target_include_directories(marker_tf2_broadcaster
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(marker_tf2_broadcaster ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(marker_tf2_broadcaster ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})

# for camera frame broadcaster

add_executable(camera_tf2_broadcaster src/camera_tf2_broadcaster.cpp)

target_include_directories(camera_tf2_broadcaster
  PUBLIC
  include)

target_include_directories(camera_tf2_broadcaster
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(camera_tf2_broadcaster ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(camera_tf2_broadcaster ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})

# for calculate_stats

add_executable(calculate_stats src/calculate_stats.cpp)
target_include_directories(calculate_stats
  PUBLIC
  include)

target_include_directories(calculate_stats
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(calculate_stats ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(calculate_stats ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})

#############
## Install ##
#############

install(TARGETS aruco_ros_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS marker_publisher single double
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)


# calculate camera coordinates

install(TARGETS
  calculate_cam_cords
  DESTINATION lib/${PROJECT_NAME})

# camera calibration

install(TARGETS 
  camera_calibration
  DESTINATION lib/${PROJECT_NAME})

# for tf broadcaster
install(TARGETS
  marker_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

# for camera frame broadcaster
install(TARGETS
  camera_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

# calculate_stats
install(TARGETS
  calculate_stats
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS 
  update_calib_points_dir_kin
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  update_calib_pts_cam
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

foreach(dir etc launch)
    install(DIRECTORY ${dir}/
            DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

ament_package()
