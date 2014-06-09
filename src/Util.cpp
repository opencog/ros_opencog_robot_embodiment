#include "Util.h"

using namespace ros_opencog_robot_embodiment;
using namespace std;
using namespace boost;
using namespace actionlib;
using namespace geometry_msgs;

//const float Util::MAP_SCALE = 0.0500000007451;

string Util::bool_to_string(bool value)
{
    if(value)
    {
        return "true";
    }
    return "false";
}

string Util::goal_state_to_str(actionlib::SimpleClientGoalState status)
{
    if(status == SimpleClientGoalState::SUCCEEDED)
    {
        return "done";
    }
    else
    {
        return "error";
    }
}

string Util::action_type_to_string(ActionType type)
{
    if(type == NAVIGATE_TO)
    {
        return "walk";
    }
    else if(type == EAT)
    {
        return "eat";
    }
    else if(type == PICKUP)
    {
        return "grab";
    }
    else
    {
        ROS_ERROR("Action type doesn't exist.");
    }
}

/**
 * Return a current time stamp with a ISO-8601 date time format.
 */

string Util::get_current_timestamp()
{
    using namespace boost::posix_time;
    ptime t = microsec_clock::local_time();
    string iso_time = to_iso_extended_string(t); //2014-05-19T08:43:33.099272
    return iso_time.substr(0, iso_time.size()-3); //2014-05-19T08:43:33.099
}

string Util::get_id()
{
    return lexical_cast<string>(boost::uuids::random_generator()());
}



