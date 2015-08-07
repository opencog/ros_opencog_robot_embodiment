#ifndef OAC_SERVER_H
#define OAC_SERVER_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <string>

#include "Constants.h"
#include "OacMessage.h"
#include <algorithm>

namespace ros_opencog_robot_embodiment
{
    class OacSubscriber
    {
        private:
            int port;
            boost::function<void(boost::shared_ptr<std::string>)> callback;
            bool reading;
            void read_data();

        public:
            OacSubscriber(boost::function<void(boost::shared_ptr<std::string>)> callback);
            ~OacSubscriber();
            void start();
            void stop();
            int get_port();
    };
}

#endif
