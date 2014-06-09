#include "Action.h"

using namespace ros_opencog_robot_embodiment;



Action::Action()
{

}

Action::~Action()
{


}

void Action::set_type(ActionType type)
{
    this->type = type;
}

ActionType Action::get_type()
{
    return this->type;
}
