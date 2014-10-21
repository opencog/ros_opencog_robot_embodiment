#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include "actions/Action.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/special_functions/round.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace ros_opencog_robot_embodiment
{
    class Util
    {
    private:

    public:
        static std::string bool_to_string(bool value);
        static std::string goal_state_to_str(actionlib::SimpleClientGoalState status);
        static std::string action_type_to_string(ActionType type);
        static std::string get_current_timestamp();
        static std::string get_id();
    };
}

#endif








