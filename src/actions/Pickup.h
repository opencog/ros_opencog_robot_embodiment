#ifndef PICKUP_ACTION_H
#define PICKUP_ACTION_H

#include "Action.h"
#include <string>

namespace ros_opencog_robot_embodiment
{
    class Pickup: public Action
    {
    protected:
        typedef Action super;
        std::string target;
    public:
        Pickup();
        void set_target(std::string target);
        std::string get_target();


        void set_action_id(int action_id);
        int get_action_id();

    };
}

 #endif
