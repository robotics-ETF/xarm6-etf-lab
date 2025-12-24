#include "real_demos/DemoNode2.h"
#include "environments/AABB.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace real_bringup
{
    class DemoNode3 : public real_bringup::DemoNode2,
                      public sim_bringup::AABB
    {
    public:
        DemoNode3(const std::string &node_name, const std::string &config_file_path);
        
        void demoCallback() override;
        int chooseObject() override;
        void computeObjectApproachAndPickPose();

    private:
        std::vector<float> object_approach_pose;
        std::vector<float> object_pick_pose;
        int obj_idx;
        float delta_z;
        float offset_z;

        enum Task
        {
            waiting_for_object,
            choosing_object,
            going_towards_object,
            picking_object,
            raising_object,
            releasing_object,
            moving_object_to_destination
        };
        Task task;

    };
}