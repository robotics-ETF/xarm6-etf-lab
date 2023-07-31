#include "move_xarm6.h"

class BottleAndGlassNode : public MoveXArm6Node
{
public:
    BottleAndGlassNode(const std::string node_name, const int period, const std::string time_unit = "milliseconds");

protected:
    void moveXArm6Callback() override { bottleAndGlassCallback(); }
    void bottleAndGlassCallback();

private:
    int sign = 1;
    const float c = M_PI / 180;
    std::vector<float> glass_angles_approach = {55*c, 35*c, -36*c, 180*c, 89*c, 0*c};
    std::vector<float> glass_angles_pick = {55*c, 41*c, -52*c, 180*c, 79*c, 0*c};
    std::vector<float> bottle_pose_pick = {640, 0, 90, 0, M_PI_2, 0};
};
