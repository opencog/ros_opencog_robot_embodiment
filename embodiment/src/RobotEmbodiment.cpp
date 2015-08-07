#include "RobotEmbodiment.h"

using namespace ros_opencog_robot_embodiment;
using namespace opencog_msgs::pai_msgs;
using namespace std;

const float RobotEmbodiment::TIMEOUT = 60.0;

RobotEmbodiment::RobotEmbodiment(string base_id, string oac_ip_address, int oac_port, string local_ip_address, int local_port)
{
    // OpenCog attributes
    this->avatar_id = "AVATAR_" + base_id;
    this->oac_id = "OAC_" + base_id;

    this->local_ip_address = boost::asio::ip::address::from_string(local_ip_address);
    this->local_port = local_port;

    this->oac_sub = new OacSubscriber(this->local_port, boost::bind(&RobotEmbodiment::oac_subscriber_callback, this, _1));
    this->oac_pub = new OacPublisher(oac_ip_address, oac_port);

    this->is_router_available = false;
    this->is_agent_loaded = false;
}

void RobotEmbodiment::start()
{
    ROS_DEBUG("STARTING ROBOT EMBODIMENT.");

    this->oac_sub->start();
    this->oac_pub->start();

    this->oac_pub->publish(OacMessage::create_oac_login_msg(this->avatar_id, this->local_ip_address, this->local_port));
    ROS_DEBUG("OAC login message sent.");

    this->oac_pub->publish(OacMessage::create_load_agent_msg(this->avatar_id, this->oac_id, "SPAWNER"));
    ROS_DEBUG("Agent load message sent.");

    this->wait_until_agent_loaded();

    this->oac_pub->publish(OacMessage::unknown_message_1());
    ROS_DEBUG("Unknown message 1 sent.");

    this->oac_pub->publish(OacMessage::unknown_message_2(this->avatar_id, this->oac_id));
    ROS_DEBUG("Unknown message 2 sent.");

    sleep(5.0);

    //send map
    //this->oac_pub->publish(OacMessage::create_fake_terrain(this->avatar_id, this->oac_id));
    this->load_map();
    ROS_DEBUG("Map message sent.");

    sleep(5.0);

    //boost::thread(&RobotEmbodiment::update_loop, this); //Start update loop

    ROS_DEBUG("Sleeping for 5 seconds.");




    ROS_DEBUG("ROBOT EMBODIMENT STARTED.");
}

void RobotEmbodiment::update_loop()
{
    ROS_DEBUG("Update loop started. Sending tick and physiological messages.");
    ros::Rate rate(2);

    while(ros::ok())
    {
        this->oac_pub->publish(OacMessage::create_tick_msg(this->avatar_id, this->oac_id));
        this->oac_pub->publish(OacMessage::create_physiological_msg(this->avatar_id, this->oac_id));
        //ros::spinOnce();
        rate.sleep();
    }

    ROS_DEBUG("Update loop stopped.");
}

void RobotEmbodiment::stop()
{
    this->oac_sub->stop();
    this->oac_pub->stop();
}


void RobotEmbodiment::wait_until_agent_loaded()
{   
    ROS_DEBUG("Waiting for agent to load.");
    ros::Rate rate(10);

    while(ros::ok() && !this->is_agent_loaded)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_DEBUG("Agent loaded.");
}

bool RobotEmbodiment::load_map()
{
    //TODO: change static_map to constant
    this->static_map_srv = this->nh.serviceClient<nav_msgs::GetMap>("static_map");
    bool map_available = ros::service::waitForService("static_map", RobotEmbodiment::TIMEOUT);

    if(!map_available)
    {
        //TODO: throw error, map service not found
        return false;
    }

    nav_msgs::GetMap srv;
    bool map_received = this->static_map_srv.call(srv);

    if(!map_received)
    {
        //TODO: throw error, map not received
        return false;
    }

    this->send_occupancy_grid("ros_opencog_robot_embodiment_map", srv.response.map);
    return true;
}


void RobotEmbodiment::oac_subscriber_callback(boost::shared_ptr<std::string> msg)
{ 
    string *msg_ptr = msg.get();

    if(RobotEmbodiment::is_command(*msg_ptr))
    {
        string command = msg_ptr->substr(1);
        this->process_oac_command(command);
    }
    else if (RobotEmbodiment::is_data(*msg_ptr))
    {
        string data = msg_ptr->substr(1);
        this->current_msg << data;
    }
    else
    {
        cout << "Unknown message type\n";
    }
}

bool RobotEmbodiment::is_command(std::string message)
{
    char selector = message[0];
    if(selector == 'c')
    {
        return true;
    }
    return false;
}

bool RobotEmbodiment::is_data(std::string message)
{
    char selector = message[0];
    if(selector == 'd')
    {
        return true;
    }
    return false;
}

void RobotEmbodiment::process_oac_command(std::string command)
{
    string temp("");
    temp = command;
    cout << "Command: " << "\"" << command << "\"\n";

    //TODO: make commands constants
    temp.erase(std::remove(temp.begin(), temp.end(), '\n'), temp.end());
    std::vector<string> tokens;    
    boost::split(tokens, temp, boost::is_any_of(" "));

    string name = tokens[0];
    if(name == "AVAILABLE_ELEMENT")
    {
        string id = tokens[1];

        if(id == "ROUTER")
        {
            this->is_router_available = true;
        }
    }
    else if(name == "UNAVAILABLE_ELEMENT")
    {
        string id = tokens[1];

        if(id == "ROUTER")
        {
            this->is_router_available = false;
        }
    }
    else if(name == "START_MESSAGE")
    {
        cout << "Start message\n";
    }
    else if(name == "NO_MORE_MESSAGES")
    {
        this->process_oac_message(this->current_msg.str());
        this->current_msg.str(string());
        cout << "Finish message\n";
    }
    else
    {
        cout << "Unsupported message\n";
    }
}

void RobotEmbodiment::process_oac_message(string message)
{
    cout << "Message: \"" << message << "\"";

    if(boost::starts_with(message, Commands::SUCCESS_LOAD))
    {
        cout << "Agent loaded.\n";
        this->is_agent_loaded = true;
    }
}

void RobotEmbodiment::send_occupancy_grid(std::string map_name, nav_msgs::OccupancyGrid map)
{
    int map_x = map.info.origin.position.x;
    int map_y = map.info.origin.position.z;
    int map_z = map.info.origin.position.y;

    int map_width = map.info.width; //x
    int map_length = map.info.height; //y
    int map_height = 2;
    int floor_height = -1;
    int num_blocks_to_transmit = 0;
    bool is_first_percept = true;

    //std::cout << "1";
    // Parse occupancy grid data into blocks
    MapInfoSeq blocks;


    for(int y = 0; y < map_length; y++)
    {
        for(int x = 0; x < map_width; x++)
        {
            int occupied_probability  = map.data[x + map_width * y];

            if(occupied_probability > 68) // TODO: constant
            {
                //std::cout << "Cell occupied: " << occupied_probability << "\n";

                MapInfo* block1 = blocks.add_mapinfos();
                Block::set_properties(block1, x, 0, y);
                num_blocks_to_transmit += 1;

                MapInfo* block2 = blocks.add_mapinfos();
                Block::set_properties(block2, x, 1, y);
                num_blocks_to_transmit += 1;
            }

            if(num_blocks_to_transmit >= OacPublisher::BLOCKS_PER_TRANSMISION)
            {
                OacMessage map_msg = OacMessage::create_map_msg(this->avatar_id, this->oac_id, map_name, map_x, map_y, map_z, map_width, map_length, map_height, floor_height, &blocks, is_first_percept);
                this->oac_pub->publish(map_msg);
                blocks.clear_mapinfos();
                num_blocks_to_transmit = 0;
            }
        }
    }

    //Send remaining blocks
    if(num_blocks_to_transmit > 0)
    {
        OacMessage map_msg = OacMessage::create_map_msg(this->avatar_id, this->oac_id, map_name, map_x, map_y, map_z, map_width, map_length, map_height, floor_height, &blocks, is_first_percept);
        this->oac_pub->publish(map_msg);
    }

    OacMessage map_perceived_msg = OacMessage::create_map_perceived_msg(this->avatar_id, this->oac_id);
    this->oac_pub->publish(map_perceived_msg);
    google::protobuf::ShutdownProtobufLibrary();
}

RobotEmbodiment::~RobotEmbodiment()
{
    this->stop();
}

//bool RobotEmbodiment::set_param(ros::NodeHandle nh, string param_id, stromg )
//{

//}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "ros_opencog_robot_embodiment");
    ros::NodeHandle rel_nh("~");

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
       ros::console::notifyLoggerLevelsChanged();
    }

    //Parse parameters
    std::cout << "Loading parameters...";
    std::string base_agent_id;
    std::string oac_ip_address;
    int oac_port;
    std::string local_ip_address;
    int local_port;

    if(rel_nh.hasParam("base_agent_id"))
    {
        rel_nh.getParam("base_agent_id", base_agent_id);
    }
    else
    {
        ROS_DEBUG("Please specify a base_agent_id in the launch file");
        return 0;
    }

    if(rel_nh.hasParam("oac_ip_address"))
    {
        rel_nh.getParam("oac_ip_address", oac_ip_address);
    }
    else
    {
        ROS_DEBUG("Please specify a oac_ip_address in the launch file");
        return 0;
    }

    if(rel_nh.hasParam("oac_port"))
    {
        rel_nh.getParam("oac_port", oac_port);
    }
    else
    {
        ROS_DEBUG("Please specify a oac_port in the launch file");
        return 0;
    }

    if(rel_nh.hasParam("local_port"))
    {
        rel_nh.getParam("local_port", local_port);
    }
    else
    {
        ROS_DEBUG("Please specify a oac_port in the launch file");
        return 0;
    }

    if(rel_nh.hasParam("local_ip_address"))
    {
        rel_nh.getParam("local_ip_address", local_ip_address);
    }
    else
    {
        ROS_DEBUG("Please specify a oac_port in the launch file");
        return 0;
    }

    std::cout << " OK\n";

    RobotEmbodiment robot_embodiment(base_agent_id, oac_ip_address, oac_port, local_ip_address, local_port);
    robot_embodiment.start();
    ros::spin();
}


/*
 *
 *
 *
 *
 *
 *
 *
void RobotEmbodiment::logout_of_router()
{
    OacMessage oac_logout_msg = OacMessage::create_oac_logout_msg(this->avatar_id);
    this->oac_pub->publish(oac_logout_msg);
}

void RobotEmbodiment::unload_agent()
{
    OacMessage unload_agent_msg = OacMessage::create_unload_agent_msg(this->avatar_id, this->oac_id, "SPAWNER");
    this->oac_pub->publish(unload_agent_msg);
}
*/
