#ifndef EAT_ACTION_H
#define EAT_ACTION_H

#include <string>
#include "Action.h"

namespace ros_opencog_robot_embodiment
{
    class Eat: public Action
    {
    protected:
        typedef Action super;
        std::string target;
    public:
        Eat();
        void set_target(std::string target);
        std::string get_target();

        void set_action_id(int action_id);
        int get_action_id();
    };
}

#endif
