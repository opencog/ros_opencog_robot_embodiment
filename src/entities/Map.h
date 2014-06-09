
#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <ros/console.h>
#include "../network/OacMessage.h"
#include "../network/OacPublisher.h"
#include "../msgs/opencog_msgs.pai_msgs.pb.h"

namespace ros_opencog_robot_embodiment
{
    class Map
    {
    private:
        ros::NodeHandle nh;
        ros::ServiceClient static_map_srv;
        std::string map_topic;
        void occupancy_grid_to_oac_map_msgs(OacPublisher* oac_pub, std::string avatar_id, std::string oac_id, nav_msgs::OccupancyGrid msg);

        std::string name;
        geometry_msgs::Point origin; //bottom left corner of map is origin
        float x_axis_length;
        float y_axis_length;
        float z_axis_length;
        float resolution;
        float floor_height;

    public:
        static const int BLOCKS_PER_TRANSMISION = 300;
        static const float TIMEOUT;
        Map(std::string map_topic);
        ~Map();
        std::string get_name();
        geometry_msgs::Point get_origin();
        float get_x_axis_length();
        float get_y_axis_length();
        float get_z_axis_length();
        float get_floor_height();

        void get_load_oac_map_msgs(OacPublisher* oac_pub, std::string avatar_id, std::string oac_id);
    };
}

#endif
