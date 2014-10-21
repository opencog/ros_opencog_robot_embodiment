#include "Robot.h"
using namespace ros_opencog_robot_embodiment;
using namespace actionlib;
using namespace geometry_msgs;
using namespace std;

//ac("fibonacci", true);

Robot::Robot()
{
    this->move_base_client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("move_base", true);

    while(!this->move_base_client->waitForServer(ros::Duration(5.0))) //TODO: make helper methods
    {
        ROS_INFO("Waiting for move_base action server");
    }
}

Robot::~Robot()
{

}

Point Robot::get_pose()
{
    tf::StampedTransform transform;

    try
    {
        this->listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        Point point;
        point.x = transform.getOrigin().x();
        point.y = transform.getOrigin().y();
        point.z = transform.getOrigin().z();
        return point;
    }
    catch (tf::TransformException e)
    {
        ROS_ERROR("%s", e.what());
    }
}

actionlib::SimpleClientGoalState::StateEnum Robot::act(Action action)
{
    Action* action_ptr = &action;

    if(NavigateTo *nav_to = dynamic_cast<NavigateTo*>(action_ptr))
    {
        return this->navigate_to(*nav_to);
    }
    else if(Pickup *pickup = dynamic_cast<Pickup*>(action_ptr))
    {
        return this->pickup(pickup->get_target());
    }
    else if(Eat *eat = dynamic_cast<Eat*>(action_ptr))
    {
        return this->eat(eat->get_target());
    }
    else
    {
        ROS_ERROR("Action doesn't exist");
        return actionlib::SimpleClientGoalState::ABORTED;
    }
}

actionlib::SimpleClientGoalState::StateEnum Robot::navigate_to(NavigateTo nav_to)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    //TODO: Convert from OpenCog to ROS coordinate system
    Point goal_point = nav_to.get_goal();
    goal.target_pose.pose.position.x = goal_point.x;
    goal.target_pose.pose.position.y = goal_point.z;
    goal.target_pose.pose.position.z = goal_point.y;
    //goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1);

    ROS_INFO("Sending goal");
    this->move_base_client->sendGoal(goal);
    this->move_base_client->waitForResult();

    if(this->move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Reached nav goal");
        return actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    else
    {
        ROS_INFO("Failed to reach nav goal");
        return actionlib::SimpleClientGoalState::ABORTED;
    }
}

actionlib::SimpleClientGoalState::StateEnum Robot::pickup(string target)
{
    sleep(2.0);
    return SimpleClientGoalState::SUCCEEDED;
}

actionlib::SimpleClientGoalState::StateEnum Robot::eat(string target)
{
    sleep(1.0);
    return SimpleClientGoalState::SUCCEEDED;
}




