
#ifndef ROBOT_EMBODIMENT_H
#define ROBOT_EMBODIMENT_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <ros/console.h>
#include <sstream>
#include "Constants.h"
#include "Util.h"
#include "network/OacSubscriber.h"
#include "network/OacPublisher.h"
#include "network/OacMessage.h"
#include "actions/ActionPlan.h"
#include "actions/Action.h"
#include "entities/Robot.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <actionlib/client/simple_action_client.h>
#include <unistd.h>
#include "entities/Map.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

namespace ros_opencog_robot_embodiment
{
	class RobotEmbodiment
	{
	private:
		ros::NodeHandle nh;
        std::string avatar_id;
        std::string oac_id;

        boost::asio::ip::address local_ip_address;
        int local_port;

        OacPublisher *oac_pub;
        OacSubscriber *oac_sub;
        void oac_subscriber_callback(boost::shared_ptr<std::string> msg);

        bool is_agent_loaded;
        //std::mutex::lock is_agent_loaded_lock;

        bool action_plan_running;
        Robot *robot;
        Map *map;



        void wait_until_agent_loaded();
        void update_loop();

        void oac_loaded_callback(bool oac_loaded);
        void action_plan_callback(ActionPlan action_plan);

	public:
        static const float TIMEOUT;
        RobotEmbodiment(std::string base_id, std::string oac_ip_address, int oac_port, std::string local_ip_address, int local_port);
        ~RobotEmbodiment();        
		void start();
        void stop();

	};
}

#endif
