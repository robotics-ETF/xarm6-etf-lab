#include "demos/MoveRealXArm6Node.h"

namespace real_bringup
{
    class BottleAndGlassNode : public real_bringup::MoveRealXArm6Node
    {
    public:
        BottleAndGlassNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void moveRealXArm6Callback() override { bottleAndGlassCallback(); }
        void bottleAndGlassCallback();

    private:
        int sign { 1 };
        const float c { M_PI / 180 };
        std::vector<float> glass_angles_approach {55*c, 35*c, -36*c, 180*c, 89*c, 0*c};
        std::vector<float> glass_angles_pick {55*c, 41*c, -52*c, 180*c, 79*c, 0*c};
        std::vector<float> bottle_pose_pick {640, 0, 90, 0, M_PI_2, 0};
        std::vector<float> current_angles, current_pose;

        enum State
        {
            going_home,
            approaching_to_bottle,
            grasping_bottle,
            raising_bottle,
            moving_towards_glass1,
            moving_towards_glass2,
            pouring_water1,
            pouring_water2,
            returning_bottle,
            releasing_bottle,
            grasping_glass,
            moving_glass1,
            moving_glass2,
            releasing_glass
        };
        State state, state_next;
    };
}