#include "OacPublisher.h"

using namespace ros_opencog_robot_embodiment;
using namespace opencog_msgs::pai_msgs;
using namespace std;

OacPublisher::OacPublisher(string oac_ip_address, int oac_port)
{
    //OAC network attributes
    this->oac_ip_address = boost::asio::ip::address::from_string(oac_ip_address);
    this->oac_port = oac_port;

    this->sender_running = false;
}

OacPublisher::~OacPublisher()
{
    this->stop();
}

void OacPublisher::start()
{
    this->publish_lock.lock();

    if(!this->sender_running)
    {
        //cout << "stating client";
        boost::system::error_code ec;
        this->endpoint = new boost::asio::ip::tcp::endpoint(this->oac_ip_address, this->oac_port);
        this->send_socket = new boost::asio::ip::tcp::socket(this->send_service, this->endpoint->protocol());
        this->send_socket->connect(*this->endpoint, ec);
        this->sender_running = true;

    }

    this->publish_lock.unlock();
}

void OacPublisher::stop()
{
    this->publish_lock.lock();

    if(this->sender_running)
    {
        boost::system::error_code ec;
        this->send_socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
        this->send_socket->close();
        this->send_service.stop();
        delete this->send_socket;
        delete this->endpoint;
        this->sender_running = false;
    }

    this->publish_lock.unlock();
}

void OacPublisher::publish(OacMessage& message)
{
    this->publish_lock.lock();

    if(this->sender_running)
    {

        boost::shared_ptr<std::string> content(new string("")); //= boost::make_shared<std::string>(message.get_content());
        content->append(message.get_content());
        //cout << "Sending message: " << *(content) << "\n";
        this->send_socket->async_send(boost::asio::buffer(*content), boost::bind(&OacPublisher::send_finished, this, content));
        boost::thread worker(&OacPublisher::run_send_service_thread, this);
    }
    else
    {
        printf("OacPublisher is not running so can't send message.\n");
    }

    this->publish_lock.unlock();
}

void OacPublisher::run_send_service_thread()
{
    this->send_service.run();
    this->send_service.reset();
}

void OacPublisher::send_finished(boost::shared_ptr<string> content)
{
    //std::cout << "send_finished \n";
}






