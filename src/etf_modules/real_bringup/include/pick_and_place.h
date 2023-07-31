#include "move_xarm6.h"

class PickAndPlaceNode : public MoveXArm6Node
{
public:
    PickAndPlaceNode(const std::string node_name, const int period, const std::string time_unit = "milliseconds");

protected:    
    void moveXArm6Callback() override { pickAndPlaceCallback(); }
    virtual void pickAndPlaceCallback();

private:
    int k = 0;
    float object_height = 27;
    float object_pick_z = 50;
    const float c = M_PI / 180;
    float delta_theta1 = 90*c;
    std::vector<float> object_angles_approach = {90*c, 33*c, -96*c, 0*c, 63*c, 0*c};

};
