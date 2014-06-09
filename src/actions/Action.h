#ifndef ACTION_H
#define ACTION_H

#include <string>

namespace ros_opencog_robot_embodiment
{
    enum ActionType
    {
        NAVIGATE_TO = 1,
        PICKUP = 2,
        EAT = 3
    };

    class Action
    {
    protected:
        ActionType type;
        int action_id;
    public:
        Action();
        virtual ~Action();
        void set_type(ActionType type);
        ActionType get_type();
    };
}

#endif
