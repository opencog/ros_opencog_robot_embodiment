#ifndef OAC_SERVER_H
#define OAC_SERVER_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <string>

#include "Constants.h"
#include "OacMessage.h"
#include <algorithm>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

namespace ros_opencog_robot_embodiment
{
    class OacSubscriber
    {
        private:
            //OpenCog attributes
            //std::string source_id, oac_id;

            int local_port;
            boost::function<void(boost::shared_ptr<std::string>)> callback;
            bool accept_running;
            boost::asio::io_service accept_service;

            boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor;
            boost::thread accept_thread;

            void handle_accept(boost::shared_ptr<boost::asio::ip::tcp::socket> socket);
            void start_accept_handler();
            void run_accept_service_thread();

        public:
            OacSubscriber(int local_port, boost::function<void(boost::shared_ptr<std::string>)> callback);
            ~OacSubscriber();
            void start();
            void stop();
    };
}

#endif
