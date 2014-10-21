#include "RobotEmbodiment.h"

using namespace ros_opencog_robot_embodiment;
using namespace opencog_msgs::pai_msgs;
using namespace actionlib;
using namespace std;
using namespace geometry_msgs;

RobotEmbodiment::RobotEmbodiment(string base_id, string oac_ip_address, int oac_port, string local_ip_address, int local_port)
{
    // OpenCog attributes
    this->avatar_id = "AVATAR_" + base_id;
    this->oac_id = "OAC_" + base_id;

    this->local_ip_address = boost::asio::ip::address::from_string(local_ip_address);
    this->local_port = local_port;

    this->oac_sub = new OacSubscriber(this->local_port);
    this->oac_sub->register_oac_loaded_callback(boost::bind(&RobotEmbodiment::oac_loaded_callback, this, _1));
    this->oac_sub->register_action_plan_callback(boost::bind(&RobotEmbodiment::action_plan_callback, this, _1));

    this->oac_pub = new OacPublisher(oac_ip_address, oac_port);

    this->is_agent_loaded = false;

    this->robot = new Robot();
    this->map = new Map("static_map");
}

void RobotEmbodiment::start()
{
    ROS_INFO("STARTING ROBOT EMBODIMENT.");

    this->oac_sub->start();
    this->oac_pub->start();

    this->oac_pub->publish(OacMessage::create_oac_login_msg(this->avatar_id, this->local_ip_address, this->local_port));
    ROS_INFO("OAC login message sent.");



    this->oac_pub->publish(OacMessage::create_load_agent_msg(this->avatar_id, this->oac_id, "SPAWNER"));
    ROS_INFO("Agent load message sent.");

    this->wait_until_agent_loaded();



    this->oac_pub->publish(OacMessage::unknown_message_1());
    ROS_INFO("Unknown message 1 sent.");

    this->oac_pub->publish(OacMessage::unknown_message_2(this->avatar_id, this->oac_id));
    ROS_INFO("Unknown message 2 sent.");

     ROS_INFO("Sending map.");

     sleep(1.0);

    //send map
    //this->oac_pub->publish(OacMessage::create_fake_terrain(this->avatar_id, this->oac_id));
    //this->oac_pub->publish(OacMessage::create_map_perceived_msg(avatar_id, oac_id));
    this->map->get_load_oac_map_msgs(this->oac_pub, this->avatar_id, this->oac_id);

     boost::thread(&RobotEmbodiment::update_loop, this); //Start update loop


    ROS_INFO("Map message sent.");

        sleep(5.0);

    MapInfoSeq objects;

    MapInfo* agent = objects.add_mapinfos();
    Point agent_pos;
    agent_pos.x = 0;
    agent_pos.y = 0;
    agent_pos.z = 0;
    OacMessage::set_agent_properties(agent, this->avatar_id, agent_pos);

    MapInfo* battery = objects.add_mapinfos();
    Point battery_pos;
    battery_pos.x = 20;
    battery_pos.y = 20;
    battery_pos.z = 0;
    OacMessage::set_battery_properties(battery, battery_pos);

    this->oac_pub->publish(OacMessage::create_map_msg(this->avatar_id, this->oac_id, this->map->get_name(), this->map->get_origin().x, this->map->get_origin().y, this->map->get_origin().z, this->map->get_x_axis_length(), this->map->get_y_axis_length(), this->map->get_z_axis_length(), this->map->get_floor_height(), &objects, false));




    ROS_INFO("Battery and agent messages sent.");

    //sleep(5.0);



   // ROS_INFO("Sleeping for 5 seconds.");

    ROS_INFO("ROBOT EMBODIMENT STARTED.");
}

void RobotEmbodiment::oac_loaded_callback(bool oac_loaded)
{
    //this->is_agent_loaded_lock.lock();
    this->is_agent_loaded = oac_loaded;
    //this->is_agent_loaded_lock.unlock();
}

void RobotEmbodiment::action_plan_callback(ActionPlan action_plan)
{
    action_plan.publish_markers();

    SimpleClientGoalState plan_state = SimpleClientGoalState::SUCCEEDED;// = true;

    for (int i = 0; i < action_plan.size(); i++)
    {
        Action action = action_plan.get_action(i);
        SimpleClientGoalState state = this->robot->act(action);
        //this->oac_pub->publish(OacMessage::create_action_result_msg(this->avatar_id, this->oac_id, action_plan.get_plan_id(), action.get_action_id(), action.get_type(), state));

        if(state != SimpleClientGoalState::SUCCEEDED)
        {
            plan_state = SimpleClientGoalState::ABORTED;
            break;
        }

        //action_plan.delete_marker(action.get_action_id());
    }

    /*for(int i = 0; i < action_plan.size(); i++)
    {
        Action action = action_plan.get_action(i);
        action_plan.delete_marker(action.get_action_id());
    }*/

    this->oac_pub->publish(OacMessage::create_action_plan_result_msg(this->avatar_id, this->oac_id, action_plan.get_plan_id(), plan_state));
}

void RobotEmbodiment::update_loop()
{
    ROS_INFO("Update loop started. Sending tick and physiological messages.");
    ros::Rate rate(2);

    while(ros::ok())
    {
        this->oac_pub->publish(OacMessage::create_tick_msg(this->avatar_id, this->oac_id));
        this->oac_pub->publish(OacMessage::create_physiological_msg(this->avatar_id, this->oac_id));


        //ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Update loop stopped.");
}

void RobotEmbodiment::stop()
{
    this->oac_sub->stop();
    this->oac_pub->stop();
}

void RobotEmbodiment::wait_until_agent_loaded()
{   
    ROS_INFO("Waiting for agent to load.");
    ros::Rate rate(10);

    while(ros::ok())
    {
        //this->is_agent_loaded_lock.lock();
        //loaded = this->is_agent_loaded;
        //this->is_agent_loaded_lock.unlock();

        if(this->is_agent_loaded)
        {
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Agent loaded.");
}

RobotEmbodiment::~RobotEmbodiment()
{
    this->stop();
    delete this->oac_sub;
    delete this->oac_pub;
    delete this->robot;
    delete this->map;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "ros_opencog_robot_embodiment");
    ros::NodeHandle rel_nh("~");

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
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
        ROS_INFO("Please specify a base_agent_id in the launch file");
        return 0;
    }

    if(rel_nh.hasParam("oac_ip_address"))
    {
        rel_nh.getParam("oac_ip_address", oac_ip_address);
    }
    else
    {
        ROS_INFO("Please specify a oac_ip_address in the launch file");
        return 0;
    }

    if(rel_nh.hasParam("oac_port"))
    {
        rel_nh.getParam("oac_port", oac_port);
    }
    else
    {
        ROS_INFO("Please specify a oac_port in the launch file");
        return 0;
    }

    if(rel_nh.hasParam("local_port"))
    {
        rel_nh.getParam("local_port", local_port);
    }
    else
    {
        ROS_INFO("Please specify a oac_port in the launch file");
        return 0;
    }

    if(rel_nh.hasParam("local_ip_address"))
    {
        rel_nh.getParam("local_ip_address", local_ip_address);
    }
    else
    {
        ROS_INFO("Please specify a oac_port in the launch file");
        return 0;
    }

    std::cout << " OK\n";

    RobotEmbodiment robot_embodiment(base_agent_id, oac_ip_address, oac_port, local_ip_address, local_port);
    robot_embodiment.start();
    ros::spin();
    //google::protobuf::ShutdownProtobufLibrary();
}


