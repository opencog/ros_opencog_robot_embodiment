#ifndef NAVIGATE_TO_ACTION_H
#define NAVIGATE_TO_ACTION_H

#include "Action.h"
#include <geometry_msgs/Point.h>
#include <vector>

namespace ros_opencog_robot_embodiment
{
    class NavigateTo: public Action
    {
    protected:
        typedef Action super;
        std::vector<geometry_msgs::Point> path;
        int action_id_start;
        int action_id_end;
    public:
        NavigateTo();
        static NavigateTo from_xml();
        geometry_msgs::Point get_goal();
        int get_action_id_start();
        int get_action_id_end();
        std::vector<geometry_msgs::Point> get_path();

        void set_action_id_range(int start, int end);
        void set_path(std::vector<geometry_msgs::Point> path);
    };
}

#endif
