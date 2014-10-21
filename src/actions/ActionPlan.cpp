#include "ActionPlan.h"

using namespace ros_opencog_robot_embodiment;
using namespace std;
using namespace boost;
using namespace geometry_msgs;

void ActionPlan::add_action(Action action)
{
    this->actions.push_back(action);
}

void ActionPlan::set_demand(string demand)
{
    this->demand = demand;
}

void ActionPlan::set_agent_id(string agent_id)
{
    this->agent_id = agent_id;
}

void ActionPlan::set_plan_id(int plan_id)
{
    this->plan_id = plan_id;
}

Action ActionPlan::get_action(int i)
{
    return this->actions[i];
}

string ActionPlan::get_demand()
{
    return this->demand;
}

string ActionPlan::get_agent_id()
{
    return this->agent_id;
}

int ActionPlan::get_plan_id()
{
    return this->plan_id;
}

int ActionPlan::size()
{
    return this->actions.size();
}

void ActionPlan::publish_markers()
{
    this->vis_pub.publish(this->markers);
}

void ActionPlan::delete_marker(int marker_id)
{
    this->markers.markers.clear();
    visualization_msgs::Marker marker;
    marker.id = marker_id;
    marker.action = visualization_msgs::Marker::DELETE;
    this->markers.markers.push_back(marker);
    this->vis_pub.publish(this->markers);
}

void ActionPlan::add_action_marker(int action_id, Point position)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = action_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = position;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    this->markers.markers.push_back(marker);
}

ActionPlan::ActionPlan()
{
    this->vis_pub = this->nh.advertise<visualization_msgs::Marker>("planner_visualisation", 0);
}

ActionPlan ActionPlan::from_xml(string xml)
{
    ActionPlan action_plan;
    const char* text = xml.c_str();
    TiXmlDocument doc;
    doc.Parse(text);

    TiXmlElement *root;
    root = doc.FirstChildElement("oc:action-plan");
    action_plan.set_demand(root->Attribute("demand"));
    action_plan.set_agent_id(root->Attribute("agent_id"));
    action_plan.set_plan_id(lexical_cast<int>(root->Attribute("plan_id")));

    TiXmlElement* child = root->FirstChildElement("action");

    string previous_type = "";
    int current_action_id;

    int action_id_start;
    vector<Point> path;
    bool parsing_navto = false;

    for(child; child; child=child->NextSiblingElement())
    {
        string current_type = child->Attribute("name");
        current_action_id = lexical_cast<int>(child->Attribute("sequence"));

        if(previous_type == "walk" && current_type != "walk" && parsing_navto)
        {
            NavigateTo action;
            action.set_path(path);
            action.set_action_id_range(action_id_start, current_action_id);
            action.set_type(NAVIGATE_TO);
            action_plan.add_action(action);
            parsing_navto = false;
            path.clear();
            //action_plan.add_action_marker(action.get_action_id(), point);
        }

        if(current_type == "walk")
        {
            if(previous_type != current_type)
            {
                parsing_navto = true;
                action_id_start = current_action_id;
            }

            TiXmlElement *vector;
            vector = child->FirstChildElement("param");
            Point point;
            point.x = lexical_cast<double>(vector->Attribute("x"));
            point.y = lexical_cast<double>(vector->Attribute("y"));
            point.z = lexical_cast<double>(vector->Attribute("z"));
            path.push_back(point);
        }
        else if(current_type == "grab")
        {
            Pickup action;
            TiXmlElement *entity;
            entity = child->FirstChild("param")->NextSibling("entity")->ToElement();
            action.set_target(entity->Attribute("id"));
            action.set_action_id(current_action_id);

            action.set_type(PICKUP);
            action_plan.add_action(action);
        }
        else if(current_type == "eat")
        {
            Eat action;
            TiXmlElement *entity;
            entity = child->FirstChild("param")->NextSibling("entity")->ToElement();
            action.set_target(entity->Attribute("id"));
            action.set_action_id(current_action_id);
            action.set_type(EAT);

            action_plan.add_action(action);
        }

        previous_type = current_type;
    }

    if(previous_type == "walk" && parsing_navto)
    {
        NavigateTo action;
        action.set_path(path);
        action.set_action_id_range(action_id_start, current_action_id);
        action.set_type(NAVIGATE_TO);
        action_plan.add_action(action);
        parsing_navto = false;
        path.clear();
    }

    return action_plan;
}
