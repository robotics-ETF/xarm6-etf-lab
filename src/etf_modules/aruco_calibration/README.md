Camera calibration procedure using Aruco marker

Before starting the procedure, following Nodes need to be ran:

make real
make cameras
ros2 run aruco_ros marker_tf2_broadcaster
PROVJERITI DA LI JE OVO SVE OKEJ I ŠTA JOŠ TREBA

Make sure that files are as following:

aruco_ros/data/calib_points.yaml (only empty objects):

transforms_dir_kin: 
transforms_camera:

aruco_ros/data/camera_coordinates_all.yaml and aruco_ros/data/camera_coordinates_final.yaml are empty files

1. Install aruco marker and align aruco marker frames in camera frame and world frame using RViz visualisation, by adjusting bias parameters in aruco_ros/data/robot_config.yaml
2. Using Ufactory, put the robot in the Manual mode
3. Move robot to random configuration (make sure that marker is visible to camera)
4. Run: make update_calib_points
5. Repeat steps 3 and 4 until you have captured desired number of points (we suggest doing it at least for 10 points to increase accuracy)
6. Run: make calculate_camera_cords

Camera calibration has finished successfully. Camera parameters (xyz_YPR format) are saved in aruco_ros/data/camera_coordinates_final.yaml.

In aruco_ros/data/camera_coordinates_all.yaml parameters of individual calculations are stored (in xyz_YPR format)

In aruco_ros/data/calib_points.yaml individual point captures are saved (in camera and world (dir_kin) frame)
