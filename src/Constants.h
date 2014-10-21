
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

namespace ros_opencog_robot_embodiment
{
	class Elements
	{
		public:
            static const std::string EMBODIMENT_MSG, TERRAIN_INFO, TERRAIN_DATA, MAP_NAME, MAP_PERCEIVED;
	};
	   //TODO: STRUCTURE_OBJECT_TYPE
	class Attributes
	{
		public:
            static const std::string XML_VERSION, XML_ENCODING, MATERIAL, MAP_POS_X, MAP_POS_Y, MAP_POS_Z, MAP_X_AXIS_LENGTH, MAP_Y_AXIS_LENGTH, MAP_Z_AXIS_LENGTH, MAP_FLOOR_HEIGHT, IS_FIRST_TIME_PERCEPT_WORLD, TIMESTAMP, SCHEMA_LOCATION, XSI, OC;
	};
	   
	class ActionStatus
	{
		public:
			// xml action status (values for <avatar-signal>'s status attribute)
			static const std::string DONE, ERROR;
	};

	class ObjectTypes
	{
		public:
			// xml object types
			static const std::string PET, AVATAR, ORDINARY, STRUCTURE, BLOCK, UNKNOWN;
	};

    class MessageType
    {
        public:
            static const int NONE = 0;
            static const int STRING = 1;
            static const int LEARN = 2;
            static const int REWARD = 3;
            static const int SCHEMA = 4;
            static const int LS_CMD = 5;
            static const int ROUTER = 6;
            static const int CANDIDATE_SCHEMA = 7;
            static const int TICK = 8;
            static const int FEEDBACK = 9;
            static const int TRY = 10;
            static const int STOP_LEARNING = 11;
            static const int RAW = 12;
    };
    
	class Commands
    {
        public:
            static const std::string AVAILABLE_ELEMENT;
			static const std::string UNAVAILABLE_ELEMENT;
			static const std::string START_MESSAGE;
			static const std::string SUCCESS_LOAD;
			static const std::string SUCCESS_UNLOAD;
			static const std::string NO_MORE_MESSAGES;        
	};
    

}

#endif








