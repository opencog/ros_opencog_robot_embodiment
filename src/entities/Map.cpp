#include "Map.h"

using namespace ros_opencog_robot_embodiment;
using namespace std;
using namespace opencog_msgs::pai_msgs;
using namespace geometry_msgs;

const float Map::TIMEOUT = 60.0;

Map::Map(string map_topic = "static_map")
{
    this->map_topic = map_topic;
}

Map::~Map()
{

}

string Map::get_name()
{
    return this->name;
}

Point Map::get_origin()
{
    return this->origin;
}

float Map::get_x_axis_length()
{
    return this->x_axis_length;
}

float Map::get_y_axis_length()
{
    return this->y_axis_length;
}

float Map::get_z_axis_length()
{
    return this->z_axis_length;
}

float Map::get_floor_height()
{
    return this->floor_height;
}

void Map::get_load_oac_map_msgs(OacPublisher* oac_pub, string avatar_id, string oac_id)
{
    //TODO: change static_map to constant
    this->static_map_srv = this->nh.serviceClient<nav_msgs::GetMap>(this->map_topic);
    bool map_available = ros::service::waitForService(this->map_topic, Map::TIMEOUT);

    if(!map_available)
    {
        ROS_ERROR("map service not found");
    }

    nav_msgs::GetMap srv;
    bool map_received = this->static_map_srv.call(srv);

    if(!map_received)
    {
        ROS_ERROR("map not received");
    }

    this->name = "ros_opencog_robot_embodiment_map";
    this->resolution = srv.response.map.info.resolution;
    this->x_axis_length = srv.response.map.info.width;
    this->y_axis_length = srv.response.map.info.height;
    this->z_axis_length = 3;
    this->floor_height = 0;

    Point opencog_map_origin; //Bottom left corner of opencog map
    opencog_map_origin.x = srv.response.map.info.origin.position.x / this->resolution;
    opencog_map_origin.y = (srv.response.map.info.origin.position.y / this->resolution) - this->y_axis_length;
    opencog_map_origin.z = 0;
    this->origin = opencog_map_origin;

    this->occupancy_grid_to_oac_map_msgs(oac_pub, avatar_id, oac_id, srv.response.map);
}

void Map::occupancy_grid_to_oac_map_msgs(OacPublisher* oac_pub, string avatar_id, string oac_id, nav_msgs::OccupancyGrid map)
{

    double global_x = map.info.origin.position.x / this->resolution;
    double global_y = map.info.origin.position.y / this->resolution;
    int num_blocks_to_transmit = 0;
    bool is_first_percept = true;

    std::cout << "1";
    // Parse occupancy grid data into blocks
    MapInfoSeq blocks;

    for(int y = 0; y < this->y_axis_length; y++)
    {
        for(int x = 0; x < this->x_axis_length ; x++)
        {
            int occupied_probability  = map.data[x + this->x_axis_length * y];

            if(occupied_probability > 65) // TODO: constant
            {
                cout << "Parsing block\n";

                MapInfo* block1 = blocks.add_mapinfos();
                string block1_id = "BLOCK_" + Util::get_id();
                Point block1_pos;
                block1_pos.x = global_x + x;
                block1_pos.y = global_y + y;
                block1_pos.z = 1;
                OacMessage::set_object_properties(block1, block1_id, ObjectTypes::STRUCTURE, block1_pos, 1.0, 1.0, 1.0, "block");
                num_blocks_to_transmit += 1;

                MapInfo* block2 = blocks.add_mapinfos();
                string block2_id = "BLOCK_" + Util::get_id();
                Point block2_pos;
                block2_pos.x = global_x + x;
                block2_pos.y = global_y + y;
                block2_pos.z = 2;
                OacMessage::set_object_properties(block2, block2_id, ObjectTypes::STRUCTURE, block2_pos, 1.0, 1.0, 1.0, "block");
                num_blocks_to_transmit += 1;
            }

            if(num_blocks_to_transmit >= Map::BLOCKS_PER_TRANSMISION)
            {

                oac_pub->publish(OacMessage::create_map_msg(avatar_id, oac_id, this->name, this->origin.x, this->origin.y, this->origin.z, this->x_axis_length, this->y_axis_length, this->z_axis_length, this->floor_height, &blocks, is_first_percept));
                blocks.clear_mapinfos();
                num_blocks_to_transmit = 0;
            }
        }        
    }

    //Send remaining blocks
    if(num_blocks_to_transmit > 0)
    {
        oac_pub->publish(OacMessage::create_map_msg(avatar_id, oac_id, this->name, this->origin.x, this->origin.y, this->origin.z, this->x_axis_length, this->y_axis_length, this->z_axis_length, this->floor_height, &blocks, is_first_percept));
    }

    oac_pub->publish(OacMessage::create_map_perceived_msg(avatar_id, oac_id));
    //google::protobuf::ShutdownProtobufLibrary();
}
