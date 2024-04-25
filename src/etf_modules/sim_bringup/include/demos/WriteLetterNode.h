#include "base/BaseNode.h"

namespace sim_bringup
{
    class WriteLetterNode : public sim_bringup::BaseNode
    {
    public:
        WriteLetterNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void baseCallback() override { WriteLetterCallback(); }
        virtual void WriteLetterCallback();

        void goHome();
        void moveInJointSpace();

    private:
        enum State
        {
            going_home,
            moving_in_joint_space
        };
        State state;
    };
}
