#include "NavigateTo.h"

using namespace ros_opencog_robot_embodiment;
using namespace std;
using namespace geometry_msgs;

NavigateTo::NavigateTo() : super()
{

}

int NavigateTo::get_action_id_start()
{
    return this->action_id_start;
}

int NavigateTo::get_action_id_end()
{
    return this->action_id_end;
}

void NavigateTo::set_action_id_range(int start, int end)
{
    this->action_id_start = start;
    this->action_id_end = end;
}

vector<Point> NavigateTo::get_path()
{
    return this->path;
}

void NavigateTo::set_path(vector<Point> path)
{
    this->path.swap(path);
}

Point NavigateTo::get_goal()
{
    return this->path.back();
}


