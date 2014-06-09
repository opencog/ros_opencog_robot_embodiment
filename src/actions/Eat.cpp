#include "Eat.h"

using namespace ros_opencog_robot_embodiment;
using namespace std;

Eat::Eat(): super()
{

}


void Eat::set_target(string target)
{
    this->target = target;
}

string Eat::get_target()
{
    return this->target;
}

void Eat::set_action_id(int action_id)
{
    this->action_id = action_id;
}

int Eat::get_action_id()
{
    return this->action_id;
}
