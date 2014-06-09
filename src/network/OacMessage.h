#ifndef OAC_MESSAGE_H
#define OAC_MESSAGE_H

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/archive/iterators/insert_linebreaks.hpp>
#include <boost/archive/iterators/remove_whitespace.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <tinyxml.h>
#include <google/protobuf/message_lite.h>
#include "../msgs/opencog_msgs.pai_msgs.pb.h"
#include "../Constants.h"
#include "../actions/Action.h"
#include <xercesc/util/Base64.hpp>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include "../Util.h"

namespace ros_opencog_robot_embodiment
{
	class OacMessage
	{
	private:
		MessageType type; // The type of the message.
		std::string content;

	public:
        OacMessage(std::string content);
        static std::string to_string(bool value);
		static std::string to_base_64(std::string string);
		std::string get_content();
        //static std::string get_current_timestamp();
        static std::string create_packet(std::string source, std::string target, int type, std::string body);
        //static std::string get_id();

        static void set_object_properties(opencog_msgs::pai_msgs::MapInfo* object, std::string id, std::string type, geometry_msgs::Point position, float length, float width, float height, std::string class_name);
        static void set_battery_properties(opencog_msgs::pai_msgs::MapInfo* battery, geometry_msgs::Point position);
        //static geometry_msgs::Point ros_to_opencog_coord(geometry_msgs::Point ros_point);
        //static geometry_msgs::Point opencog_to_ros_coord(geometry_msgs::Point opencog_point);
        static void set_agent_properties(opencog_msgs::pai_msgs::MapInfo* agent, std::string agent_id, geometry_msgs::Point position);

        static std::string action_type_to_string(ActionType type);
        //static std::string goal_state_to_str(actionlib::SimpleClientGoalState status);

        static OacMessage create_oac_login_msg(std::string avatar_id, boost::asio::ip::address local_ip_address, int local_port);
        static OacMessage create_load_agent_msg(std::string avatar_id, std::string oac_id, std::string spawner_id);
        static OacMessage unknown_message_1();
        static OacMessage unknown_message_2(std::string avatar_id, std::string oac_id);
        static OacMessage create_tick_msg(std::string avatar_id, std::string oac_id);
        static OacMessage create_physiological_msg(std::string avatar_id, std::string oac_id);
        static OacMessage create_fake_terrain(std::string avatar_id, std::string oac_id);

        static OacMessage create_map_msg(std::string avatar_id, std::string oac_id, std::string map_name, int map_x, int map_y, int map_z, int x_axis_length, int y_axis_length, int z_axis_length, int floor_height, opencog_msgs::pai_msgs::MapInfoSeq* blocks, bool is_first_time_percept_world);
        static OacMessage create_map_perceived_msg(std::string avatar_id, std::string oac_id);

        static OacMessage create_action_result_msg(std::string avatar_id, std::string oac_id, int plan_id, int action_id, ActionType type, actionlib::SimpleClientGoalState status);
        static OacMessage create_action_plan_result_msg(std::string avatar_id, std::string oac_id, int plan_id, actionlib::SimpleClientGoalState status);
	};
}

#endif
