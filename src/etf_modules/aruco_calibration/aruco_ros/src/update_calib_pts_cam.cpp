#include "./aruco_ros/update_calib_points_cam.h"

std::string camera_side;
std::vector<double> translation_x, translation_y, translation_z;
std::vector<double> rotation_x, rotation_y, rotation_z, rotation_w;
int point_count = 0;
const int POINTS_TO_COLLECT = 100;  // Number of points to average

TransformUpdateNode::TransformUpdateNode()
    : Node("transform_update_node")
{
    this->declare_parameter<std::string>("camera_side", "left");
    camera_side = this->get_parameter("camera_side").get_value<std::string>();

    subscription_camera = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "aruco_single/transform", 10,
        std::bind(&TransformUpdateNode::camera_callback, this, std::placeholders::_1));
}

void TransformUpdateNode::camera_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    // Store the translation and rotation values in the vectors
    translation_x.push_back(msg->transform.translation.x);
    translation_y.push_back(msg->transform.translation.y);
    translation_z.push_back(msg->transform.translation.z);

    rotation_x.push_back(msg->transform.rotation.x);
    rotation_y.push_back(msg->transform.rotation.y);
    rotation_z.push_back(msg->transform.rotation.z);
    rotation_w.push_back(msg->transform.rotation.w);

    point_count++;

    

    // Once we have collected the desired number of points, compute the average
    if (point_count == POINTS_TO_COLLECT) {


        // Calculate the average translation
        double avg_translation_x = std::accumulate(translation_x.begin(), translation_x.end(), 0.0) / POINTS_TO_COLLECT;
        double avg_translation_y = std::accumulate(translation_y.begin(), translation_y.end(), 0.0) / POINTS_TO_COLLECT;
        double avg_translation_z = std::accumulate(translation_z.begin(), translation_z.end(), 0.0) / POINTS_TO_COLLECT;

        // Calculate the average rotation
        double avg_rotation_x = std::accumulate(rotation_x.begin(), rotation_x.end(), 0.0) / POINTS_TO_COLLECT;
        double avg_rotation_y = std::accumulate(rotation_y.begin(), rotation_y.end(), 0.0) / POINTS_TO_COLLECT;
        double avg_rotation_z = std::accumulate(rotation_z.begin(), rotation_z.end(), 0.0) / POINTS_TO_COLLECT;
        double avg_rotation_w = std::accumulate(rotation_w.begin(), rotation_w.end(), 0.0) / POINTS_TO_COLLECT;

        RCLCPP_INFO(this->get_logger(), "\nUsrednjena vrijednost: ");
        RCLCPP_INFO(this->get_logger(), "%f", avg_translation_x);
        RCLCPP_INFO(this->get_logger(), "%f", avg_translation_y);
        RCLCPP_INFO(this->get_logger(), "%f", avg_translation_z);

        // Prepare the average data for YAML
        YAML::Node transform_data;
        transform_data["translation"]["x"] = avg_translation_x;
        transform_data["translation"]["y"] = avg_translation_y;
        transform_data["translation"]["z"] = avg_translation_z;
        transform_data["rotation"]["x"] = avg_rotation_x;
        transform_data["rotation"]["y"] = avg_rotation_y;
        transform_data["rotation"]["z"] = avg_rotation_z;
        transform_data["rotation"]["w"] = avg_rotation_w;

        std::string project_abs_path(__FILE__);
        for (size_t i = 0; i < 4; i++)
            project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
        
        const std::string config_file_path = "/aruco_calibration/aruco_ros/data/" + camera_side + "_camera/calib_points.yaml";
        
        YAML::Node yaml_data;
        std::string yaml_file_path = project_abs_path + config_file_path;
        
        // Load existing data from the YAML file
        try {
            if (std::ifstream yaml_file(yaml_file_path); yaml_file.is_open()) {
                yaml_data = YAML::Load(yaml_file);
            }

            // Ensure 'transforms_camera' key exists and is a sequence
            if (!yaml_data["transforms_camera"]) {
                yaml_data["transforms_camera"] = YAML::Node(YAML::NodeType::Sequence);
            }

            // Append the new averaged transform data to the list
            yaml_data["transforms_camera"].push_back(transform_data);

            // Write the updated data back to the YAML file
            std::ofstream yaml_out(yaml_file_path);
            yaml_out << yaml_data;

            RCLCPP_INFO(this->get_logger(), "YAML file updated with averaged transform data.");

            // Shutdown the node after the update
            rclcpp::shutdown();
        }
        catch (...) {
            std::cout << "Problem accessing YAML file.\n";
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformUpdateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
