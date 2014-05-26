#ifndef OAC_CLIENT_H
#define OAC_CLIENT_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <string>

#include "Constants.h"
#include "OacMessage.h"
#include <algorithm>

namespace ros_opencog_robot_embodiment
{
    class OacPublisher
    {
        private:
            //OpenCog attributes
            std::string source_id, oac_id;

            //OAC network attributes
            boost::asio::ip::address oac_ip_address;
            int oac_port;
            boost::asio::ip::tcp::endpoint* endpoint;

            //Sender
            boost::asio::io_service send_service;
            boost::asio::ip::tcp::socket* send_socket;
            boost::thread send_thread;
            bool sender_running;
            boost::mutex publish_lock;
            void run_send_service_thread();
            void send_finished(boost::shared_ptr<std::string> content);

        public:
            static const int BLOCKS_PER_TRANSMISION = 4096;
            OacPublisher(std::string oac_ip_address, int oac_port);
            ~OacPublisher();
            void start();
            void stop();
            void publish(OacMessage& message);
    };
}

#endif

