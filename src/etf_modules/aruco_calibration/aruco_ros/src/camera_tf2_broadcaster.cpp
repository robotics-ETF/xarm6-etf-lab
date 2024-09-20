
/*

    Class does the following:
    1. Subscribes to /xarm6_traj_controller/state to get the coordinates of the final robot link
    2. Adds bias, than publishes tf2 so that it can be seen in gazebo
    3. Creates dir_kin_aruco_pos topic, publishes aruco marker frame coordinates calculated using direct kinematics
*/

#include "./aruco_ros/camera_tf2_broadcaster.h"

using namespace std::chrono_literals;

geometry_msgs::msg::TransformStamped loadTransformFromYAML(const std::string& filename) {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile(filename);

    // Extract the xyz_YPR array
    std::vector<double> xyz_YPR = config["xyz_YPR"].as<std::vector<double>>();

    // Ensure the array has exactly 6 elements (x, y, z, yaw, pitch, roll)
    if (xyz_YPR.size() != 6) {
        throw std::runtime_error("Invalid number of elements in xyz_YPR.");
    }

    // Initialize the TransformStamped message
    geometry_msgs::msg::TransformStamped transform;
    transform.transform.translation.x = xyz_YPR[0];
    transform.transform.translation.y = xyz_YPR[1];
    transform.transform.translation.z = xyz_YPR[2];

    // Convert Yaw, Pitch, Roll to Quaternion
    tf2::Quaternion q;
    q.setRPY(xyz_YPR[5], xyz_YPR[4], xyz_YPR[3]);  // Roll, Pitch, Yaw
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    return transform;
}


void printKDLFrame(const KDL::Frame& frame, const rclcpp::Logger& logger)
{
    // Extract the translation part
    KDL::Vector translation = frame.p;
    
    // Extract the rotation part as a rotation matrix
    KDL::Rotation rotation = frame.M;
    double roll, pitch, yaw;
    rotation.GetRPY(roll, pitch, yaw);

    // Print the translation components
    RCLCPP_INFO(logger, "Translation: x = %f, y = %f, z = %f", translation.x(), translation.y(), translation.z());
    
    // Print the rotation components (in Roll-Pitch-Yaw format)
    RCLCPP_INFO(logger, "Rotation: roll = %f, pitch = %f, yaw = %f", roll, pitch, yaw);
}

KDL::Frame transformStampedToKDLFrame(const geometry_msgs::msg::TransformStamped& transform)
{
    KDL::Vector translation(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    );

    KDL::Rotation rotation = KDL::Rotation::Quaternion(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    );

    return KDL::Frame(rotation, translation);
}

geometry_msgs::msg::Transform kdlFrameToTransform(const KDL::Frame& frame)
{
    geometry_msgs::msg::Transform transform;

    transform.translation.x = frame.p.x();
    transform.translation.y = frame.p.y();
    transform.translation.z = frame.p.z();

    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    transform.rotation.x = x;
    transform.rotation.y = y;
    transform.rotation.z = z;
    transform.rotation.w = w;

    return transform;
}

geometry_msgs::msg::TransformStamped transformFrameInWorld(const KDL::Frame& frame)
{

    KDL::Frame tmp_frame = frame;
    
    KDL::Rotation rotation_y = KDL::Rotation::RotY(-M_PI / 2.0);
    KDL::Rotation rotation_x = KDL::Rotation::RotX(M_PI / 2.0);
    
    tmp_frame.M = tmp_frame.M * rotation_y;
    tmp_frame.M = tmp_frame.M * rotation_x;

    KDL::Frame robot_marker_kin = tmp_frame;

    geometry_msgs::msg::TransformStamped t;
    
    t = tf2::kdlToTransform(robot_marker_kin);

    return t;
}


FramePublisher::FramePublisher(const std::string config_file_path)
  : Node("marker_tf2_frame_publisher")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->declare_parameter<std::string>("camera_side", "left");
    camera_side = this->get_parameter("camera_side").get_value<std::string>();
    
    camera_position_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("camera_" + camera_side + "_in_world", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&FramePublisher::DirKinArucoPosCallback, this));
    
  }

// calculates coordiinates of the final link, adds bias, publishes marker frame
void FramePublisher::jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{   
    std::string project_abs_path(__FILE__);
    for (size_t i = 0; i < 4; i++)
		project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    const std::string camera_coordinates_file_path_final = project_abs_path + "/aruco_calibration/aruco_ros/data/" + camera_side + "_camera/camera_coordinates_final.yaml";
    

    try {
        geometry_msgs::msg::TransformStamped t = loadTransformFromYAML(camera_coordinates_file_path_final);

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "camera_" + camera_side + "_calculated";    

        tf_broadcaster_->sendTransform(t);    
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
 
  // t = transformFrameInWorld(frame);
   
}

void FramePublisher::DirKinArucoPosCallback(){


  std::string project_abs_path(__FILE__);
    for (size_t i = 0; i < 4; i++)
		project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    const std::string camera_coordinates_file_path_final = project_abs_path + "/aruco_calibration/aruco_ros/data/" + camera_side + "_camera/camera_coordinates_final.yaml";
    

    try {
        geometry_msgs::msg::TransformStamped t = loadTransformFromYAML(camera_coordinates_file_path_final);

        /*
        KDL::Frame t_frame = transformStampedToKDLFrame(t);

        t = transformFrameInWorld(t_frame);
        */
       
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "camera_" + camera_side + "_calculated";    

        printKDLFrame(transformStampedToKDLFrame(t), this->get_logger());

        tf_broadcaster_->sendTransform(t);    
        
        camera_position_pub->publish(t);    
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }  
}

int main(int argc, char * argv[])
{

  const std::string config_file_path = "/aruco_calibration/aruco_ros/data/robot_config.yaml";

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>(config_file_path));
  rclcpp::shutdown();
  return 0;

}