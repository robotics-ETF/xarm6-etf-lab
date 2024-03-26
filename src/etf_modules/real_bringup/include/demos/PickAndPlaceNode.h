#include "demos/MoveRealXArm6Node.h"

namespace real_bringup
{
    class PickAndPlaceNode : public real_bringup::MoveRealXArm6Node
    {
    public:
        PickAndPlaceNode(const std::string &node_name, const std::string &config_file_path);

    protected:    
        void moveRealXArm6Callback() override { pickAndPlaceCallback(); }
        virtual void pickAndPlaceCallback();

        std::vector<float> current_angles, current_pose;

    private:
        int num { 0 };
        int num_objects { 3 };
        float object_height { 27 };   // in [mm]
        float object_pick_z { 50 };   // in [mm]
        const float c { M_PI / 180 };
        float delta_theta1 { 90*c };
        std::vector<float> object_angles_approach {90*c, 33*c, -96*c, 0*c, 63*c, 0*c};

        enum Task
        {
            going_home,
            approaching_to_object,
            lowering_robot,
            grasping_object,
            raising_object,
            moving_object_to_goal,
            lowering_object,
            releasing_object,
            raising_robot
        };
        Task task;
    };
}