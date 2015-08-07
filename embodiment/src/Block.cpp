#include "Block.h"

using namespace ros_opencog_robot_embodiment;
using namespace opencog_msgs::pai_msgs;
using namespace std;

string Block::get_id()
{
    return boost::lexical_cast<string>(boost::uuids::random_generator()());
}

void Block::set_properties(MapInfo* block, float x, float y, float z)
{
    string block_name = "BLOCK_" + Block::get_id();

    block->set_id(block_name);
    block->set_name(block_name);
    block->set_type(ObjectTypes::STRUCTURE);

    MapInfo_Vector3* position = block->mutable_position();
    position->set_x(x);
    position->set_y(y);
    position->set_z(z);


    MapInfo_Rotation* rotation = block->mutable_rotation();
    rotation->set_pitch(0.0);
    rotation->set_roll(0.0);
    rotation->set_yaw(0.0);

    block->set_length(1.0);
    block->set_width(1.0);
    block->set_height(1.0);

    MapInfo_OCProperty* class_prop = block->add_properties();
    class_prop->set_key("class");
    class_prop->set_value("block");

    MapInfo_OCProperty* visi_prop = block->add_properties();
    visi_prop->set_key("visibility-status");
    visi_prop->set_value("visible");


    MapInfo_OCProperty* detector_prop = block->add_properties();
    detector_prop->set_key("detector");
    detector_prop->set_value("true");

    MapInfo_OCProperty* material_prop = block->add_properties();
    material_prop->set_key(Attributes::MATERIAL);
    material_prop->set_value("");
}
