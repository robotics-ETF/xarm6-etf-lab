#include "base/BaseNode.h"

namespace sim_bringup
{
    class DrawNode : public sim_bringup::BaseNode
    {
    public:
        DrawNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void baseCallback() override { DrawCallback(); }
        virtual void DrawCallback();

        void goHome();
        bool moveInJointSpace();

    private:
        enum State
        {
            going_home,
            moving_in_joint_space
        };
        State state;
    };
}
