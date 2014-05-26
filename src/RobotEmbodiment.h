
#ifndef ROBOT_EMBODIMENT_H
#define ROBOT_EMBODIMENT_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <vector>

#include <sstream>
#include "Constants.h"
#include "OacSubscriber.h"
#include "OacPublisher.h"
#include "OacMessage.h"
#include "opencog_msgs.pai_msgs.pb.h"
#include "Block.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

namespace ros_opencog_robot_embodiment
{
	class RobotEmbodiment
	{
	private:
		ros::NodeHandle nh;
		ros::ServiceClient static_map_srv;
        std::string avatar_id;
        std::string oac_id;

        boost::asio::ip::address local_ip_address;
        int local_port;

        OacPublisher *oac_pub;
        OacSubscriber *oac_sub;
        void oac_subscriber_callback(boost::shared_ptr<std::string> msg);

        bool is_router_available;
        bool is_agent_loaded;
        std::stringstream current_msg;

        void login_to_router();
        void logout_of_router();
        void load_agent();
        void unload_agent();
        bool load_map();
        bool is_command(std::string message);
        bool is_data(std::string message);
        void process_oac_message(std::string message);
        void process_oac_command(std::string command);
	public:
        static const float TIMEOUT;
        RobotEmbodiment(std::string base_id, std::string oac_ip_address, int oac_port, std::string local_ip_address, int local_port);
        ~RobotEmbodiment();        
		void start();
        void stop();
		void send_occupancy_grid(std::string map_name, nav_msgs::OccupancyGrid msg);
	};
}

#endif
