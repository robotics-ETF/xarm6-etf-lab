#include "base.h"

class MoveXArm6Node : public BaseNode
{
public:
    MoveXArm6Node(const std::string node_name, const int period, const std::string time_unit = "milliseconds");

protected:
    void baseCallback() override { moveXArm6Callback(); }
    virtual void moveXArm6Callback();
    void goHome();
    void moveInJointSpace();
};
