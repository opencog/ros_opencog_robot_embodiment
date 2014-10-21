#include "Pickup.h"

using namespace ros_opencog_robot_embodiment;
using namespace std;

Pickup::Pickup(): super()
{

}

void Pickup::set_target(string target)
{
    this->target = target;
}

string Pickup::get_target()
{
    return this->target;
}

void Pickup::set_action_id(int action_id)
{
    this->action_id = action_id;
}

int Pickup::get_action_id()
{
    return this->action_id;
}

