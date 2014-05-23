#ifndef OAC_MESSAGE_H
#define OAC_MESSAGE_H

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/archive/iterators/insert_linebreaks.hpp>
#include <boost/archive/iterators/remove_whitespace.hpp>


#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <tinyxml.h>
#include <google/protobuf/message_lite.h>
#include "opencog_msgs.pai_msgs.pb.h"
#include "Constants.h"

namespace ros_opencog_robot_embodiment
{
	class OacMessage
	{
	private:
		//std::string source_id; // The ID of source OCNetworkElement.
		//std::string target_id; // The ID of the target OCNetworkElement.
		MessageType type; // The type of the message.
		std::string content;

	public:
        OacMessage(std::string content);
		template<typename T>
		static std::string to_string(const T& value);
		static std::string to_base_64(std::string string);
		std::string get_content();
        static OacMessage create_tick_msg(std::string avatar_id, std::string oac_id);

		static std::string get_current_timestamp();
        static OacMessage create_fake_terrain(std::string avatar_id, std::string oac_id);
        static OacMessage create_load_agent_msg(std::string avatar_id, std::string oac_id, std::string spawner_id);
        static OacMessage create_unload_agent_msg(std::string avatar_id, std::string oac_id, std::string spawner_id);
        static OacMessage create_oac_login_msg(std::string avatar_id, boost::asio::ip::address local_ip_address, int local_port);
        static OacMessage create_oac_logout_msg(std::string avatar_id);
        static OacMessage create_map_msg(std::string avatar_id, std::string oac_id, std::string map_name, int map_x, int map_y, int map_z, int map_width, int map_length, int map_height, int floor_height, opencog_msgs::pai_msgs::MapInfoSeq* blocks, bool is_first_time_percept_world);
        static OacMessage create_map_perceived_msg(std::string avatar_id, std::string oac_id);
	};
}

#endif
