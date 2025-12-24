#include "real_demos/MoveXArm6Node.h"

namespace real_bringup
{
    class DemoNode2 : public real_bringup::MoveXArm6Node
    {
    public:
        DemoNode2(const std::string &node_name, const std::string &config_file_path);

    protected:    
        void moveXArm6Callback() override { demoCallback(); }
        virtual void demoCallback();

    private:
        size_t num;
        size_t num_objects;
        float object_height;
        float object_pick_z;
        float delta_theta1;
        std::vector<float> object_angles_approach;

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