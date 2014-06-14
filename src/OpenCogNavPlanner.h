#ifndef OPENCOG_NAV_PLANNER_H
#define OPENCOG_NAV_PLANNER_H

#include <string>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include "actions/Action.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/special_functions/round.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace ros_opencog_robot_embodiment
{
    class OpenCogNavPlanner
    {
    private:


    public:
		OpenCogNavPlanner()
		OpenCogNavPlanner()
		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
		bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
		double footprintCost(double x_i, double y_i, double theta_i);
    };
}

#endif








