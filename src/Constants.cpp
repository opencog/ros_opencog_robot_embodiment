#include "Constants.h"

using namespace ros_opencog_robot_embodiment;
using namespace std;

const string Elements::EMBODIMENT_MSG = "oc:embodiment-msg";
const string Elements::TERRAIN_INFO = "terrain-info";
const string Elements::TERRAIN_DATA = "terrain-data";
const string Elements::MAP_NAME = "map-name";
const string Elements::MAP_PERCEIVED = "finished-first-time-percept-terrian-signal";


const string Attributes::XML_VERSION = "1.0";
const string Attributes::XML_ENCODING = "UTF-8";

const string Attributes::MAP_POS_X = "global-position-x";
const string Attributes::MAP_POS_Y = "global-position-y";
const string Attributes::MAP_POS_Z = "global-position-z";
const string Attributes::MAP_X_AXIS_LENGTH = "global-position-offset-x";
const string Attributes::MAP_Y_AXIS_LENGTH = "global-position-offset-y";
const string Attributes::MAP_Z_AXIS_LENGTH = "global-position-offset-z";

const string Attributes::MAP_FLOOR_HEIGHT = "global-floor-height";
const string Attributes::IS_FIRST_TIME_PERCEPT_WORLD = "is-first-time-percept-world";
const string Attributes::TIMESTAMP = "timestamp";
const string Attributes::SCHEMA_LOCATION = "xsi:schemaLocation";
const string Attributes::XSI = "xmlns:xsi";
const string Attributes::OC = "xmlns:oc";
const string Attributes::MATERIAL = "material";

const string ActionStatus::DONE = "done";
const string ActionStatus::ERROR = "error";

const string ObjectTypes::PET = "pet";
const string ObjectTypes::AVATAR = "avatar";
const string ObjectTypes::ORDINARY = "object";
const string ObjectTypes::STRUCTURE = "structure";
const string ObjectTypes::BLOCK = "block-entity";
const string ObjectTypes::UNKNOWN = "unknown";

const string Commands::AVAILABLE_ELEMENT = "AVAILABLE_ELEMENT";
const string Commands::UNAVAILABLE_ELEMENT = "UNAVAILABLE_ELEMENT";
const string Commands::START_MESSAGE = "START_MESSAGE";
const string Commands::SUCCESS_LOAD = "SUCCESS LOAD";
const string Commands::SUCCESS_UNLOAD = "SUCCESS UNLOAD";
const string Commands::NO_MORE_MESSAGES = "NO_MORE_MESSAGES";



