#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <unistd.h>
#include <actionlib/client/simple_client_goal_state.h>
#include "../actions/Action.h"
#include "../actions/NavigateTo.h"
#include "../actions/Pickup.h"
#include "../actions/Eat.h"
#include <tf/transform_listener.h>

namespace ros_opencog_robot_embodiment
{
    //typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    class Robot
    {
    private:
        ros::NodeHandle nh;        
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* move_base_client;

        actionlib::SimpleClientGoalState::StateEnum navigate_to(NavigateTo navto);
        actionlib::SimpleClientGoalState::StateEnum pickup(std::string target);
        actionlib::SimpleClientGoalState::StateEnum eat(std::string target);

        tf::TransformListener listener;

    public:
        Robot();
        ~Robot();
        geometry_msgs::Point get_pose();
        actionlib::SimpleClientGoalState::StateEnum act(Action action);


    };
}

#endif
