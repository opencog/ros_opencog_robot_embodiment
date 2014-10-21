#ifndef OAC_SERVER_H
#define OAC_SERVER_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <ros/console.h>
#include <string>
#include <iostream>
#include <fstream>

#include "../Constants.h"
#include "OacMessage.h"
#include <algorithm>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include "../actions/ActionPlan.h"
#include<iostream>
#include<istream>

namespace ros_opencog_robot_embodiment
{
    typedef boost::function<void(bool)> OacLoadedCallback;
    typedef boost::function<void(ActionPlan)> ActionPlanCallback;

    class OacSubscriber
    {
        private:
            //OpenCog attributes
            //std::string source_id, oac_id;

            int local_port;
            //boost::function<void(boost::shared_ptr<std::string>)> callback;
            bool accept_running;
            boost::asio::io_service accept_service;

            boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor;
            boost::thread accept_thread;

            void handle_accept(boost::shared_ptr<boost::asio::ip::tcp::socket> socket);
            void start_accept_handler();
            void run_accept_service_thread();

            std::stringstream current_msg;
            OacLoadedCallback oac_loaded_callback;
            ActionPlanCallback action_plan_callback;

            void process_line(std::string line);
            void process_oac_command(std::string command);

        public:
            OacSubscriber(int local_port);
            ~OacSubscriber();
            void fire_message_callbacks(std::string message);
            void register_oac_loaded_callback(OacLoadedCallback oac_loaded_callback);
            void register_action_plan_callback(ActionPlanCallback action_plan_callback);
            void start();
            void stop();
    };
}

#endif
