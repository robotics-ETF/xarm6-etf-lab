#include "turtle_tf2_broadcaster.h"

using namespace std::chrono_literals;


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
    

    KDL::Rotation rotation_z = KDL::Rotation::RotZ(M_PI / 2.0);
    KDL::Rotation rotation_x = KDL::Rotation::RotX(-M_PI / 2.0);

    tmp_frame.M = tmp_frame.M * rotation_z;
    tmp_frame.M = tmp_frame.M * rotation_x;

    KDL::Frame robot_marker_kin = tmp_frame;

    geometry_msgs::msg::TransformStamped t;
    
    t = tf2::kdlToTransform(robot_marker_kin);

    return t;
}




FramePublisher::FramePublisher()
  : Node("turtle_tf2_frame_publisher")
  {
    // Declare and acquire `turtlename` parameter
    // turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    std::ostringstream stream;
    // stream << "/" << turtlename_.c_str() << "/pose";
    std::string topic_name = "/aruco_single/transform";

    
    subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    topic_name, 10,
    std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));

    joints_state_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
		("/xarm6_traj_controller/state", 10, std::bind(&FramePublisher::jointsStateCallback, this, std::placeholders::_1));

    aruco_position_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("dir_kin_aruco_pos", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&FramePublisher::DirKinArucoPosCallback, this));
    
    const std::string config_file_path = "/aruco_calibration/aruco_ros/data/robot_config.yaml";

	  std::string project_abs_path(__FILE__);
	  for (size_t i = 0; i < 4; i++)
		  project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
	  std::cout << "Path to URDF: " << project_abs_path + config_file_path << "\n";

    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    robot = std::make_shared<robots::xArm6>(project_abs_path + node["robot"]["urdf"].as<std::string>());

	  joints_position = Eigen::VectorXf::Zero(robot->getNumDOFs());
	  aruco_bias = Eigen::Vector3f::Zero();
	  for (size_t i = 0; i < 3; i++)
		  aruco_bias(i) = node["aruco"]["bias"][i].as<float>();
    
  }
void FramePublisher::jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    for (size_t i = 0; i < robot->getNumDOFs(); i++)
        joints_position(i) = msg->actual.positions[i];

	frame = robot->computeForwardKinematics(std::make_shared<base::RealVectorSpaceState>(joints_position))->back();

  
	frame.p += aruco_bias.x() * frame.M.UnitX();
	frame.p += aruco_bias.y() * frame.M.UnitY();
	frame.p += aruco_bias.z() * frame.M.UnitZ();

  geometry_msgs::msg::TransformStamped t;

  t = transformFrameInWorld(frame);
   
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "nas_marker_frame";

  printKDLFrame(transformStampedToKDLFrame(t), this->get_logger());
  
  tf_broadcaster_->sendTransform(t);
}

void FramePublisher::handle_turtle_pose(const std::shared_ptr<geometry_msgs::msg::TransformStamped> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables	  // KDL::Rotation rot = frame.M; 	// orijentacija
	  // KDL::Vector pos = frame.p;		// pozicija
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nas_frame";
    t.child_frame_id = "world";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = 1;
    t.transform.translation.y = 1;
    t.transform.translation.z = 1;

    
    
    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtainnas_frame
    // rotation in z axis from the message
    
    /*
    tf2::Quaternion q;
    

    q.setRPY(0, 0, msg->theta);
    */
    t.transform.rotation.x = 1;
    t.transform.rotation.y = 1;
    t.transform.rotation.z = 1;
    t.transform.rotation.w = 1;
    
    // RCLCPP_INFO(this->get_logger(), "RADI FIKSNI FRAME");
    // Send the transformation
    // tf_broadcaster_->sendTransform(t);
  }


void FramePublisher::DirKinArucoPosCallback(){
  
  geometry_msgs::msg::TransformStamped t;

  t = transformFrameInWorld(frame);

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "nas_marker_frame";

  aruco_position_pub->publish(t);

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;

}