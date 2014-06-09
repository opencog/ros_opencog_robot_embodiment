#ifndef ACTION_PLAN_H
#define ACTION_PLAN_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include "Action.h"
#include "NavigateTo.h"
#include "Pickup.h"
#include "Eat.h"
#include "../network/OacMessage.h"
#include <tinyxml.h>
#include <boost/lexical_cast.hpp>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ros_opencog_robot_embodiment
{
    class ActionPlan
    {
    private:
        std::vector<Action> actions;
        std::string demand;
        std::string agent_id;
        int plan_id;
        visualization_msgs::MarkerArray markers;
        ros::Publisher vis_pub;
        ros::NodeHandle nh;


    public:
        ActionPlan();
        void add_action(Action action);
        void set_demand(std::string demand);
        void set_agent_id(std::string agent_id);
        void set_plan_id(int plan_id);
        void delete_marker(int marker_id);
        void add_action_marker(int action_id, geometry_msgs::Point position);
        void publish_markers();
        Action get_action(int i);
        std::string get_demand();
        std::string get_agent_id();
        int get_plan_id();

        int size();
        static ActionPlan from_xml(std::string xml);
    };
}

#endif
