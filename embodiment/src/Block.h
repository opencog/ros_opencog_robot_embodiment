#ifndef BLOCK_H
#define BLOCK_H

#include <string>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>
#include "opencog_msgs.pai_msgs.pb.h"
#include "Constants.h"

namespace ros_opencog_robot_embodiment
{	
	class Block
	{
	public:
		static std::string get_id();
		static void set_properties(opencog_msgs::pai_msgs::MapInfo* block, float x, float y, float z);
	};
}

#endif
