#include "OacSubscriber.h"

using namespace ros_opencog_robot_embodiment;
using namespace std;

OacSubscriber::OacSubscriber(int local_port, boost::function<void(boost::shared_ptr<std::string>)> callback)
{
    this->local_port = local_port;
    this->callback = callback;
    this->accept_running = false;
}

OacSubscriber::~OacSubscriber()
{
    this->stop();
}

void OacSubscriber::start()
{
    if(!this->accept_running)
    {
        this->acceptor = boost::shared_ptr<boost::asio::ip::tcp::acceptor>(new boost::asio::ip::tcp::acceptor(this->accept_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), this->local_port)));
        this->start_accept_handler();
        this->accept_thread = boost::thread(&OacSubscriber::run_accept_service_thread, this);
        this->accept_running = true;
    }
}

void OacSubscriber::stop()
{
    if(this->accept_running)
    {
        this->acceptor->cancel();
        this->accept_service.stop();
        this->accept_running = false;
    }
}

void OacSubscriber::handle_accept(boost::shared_ptr<boost::asio::ip::tcp::socket> socket)
{
    cout << "Socket connected\n";


    try
    {
        boost::asio::streambuf buffer;

        while(true)
        {
            typedef boost::asio::streambuf::const_buffers_type buffer_type;
            typedef boost::asio::buffers_iterator<buffer_type> iterator;
            boost::system::error_code err;
            int length = boost::asio::read_until(*socket, buffer, "\n", err);

            if (err)
            {
                std::cerr << "Error: " <<  err << "\n";
                break;
            }
            else
            {
                //cout << "Read line successfully. Length: " << length << ", data: ";
                //boost::shared_ptr<string> line(new string(""));
                string data;
                buffer_type input = buffer.data();
                std::copy(iterator::begin(input), iterator::end(input), std::back_inserter(data));


                string temp("");
                temp = data;
                std::vector<string> lines;
                boost::split(lines, temp, boost::is_any_of("\n"));

                for(int i = 0; i < lines.size(); i++)
                {
                    boost::shared_ptr<string> msg(new std::string(""));
                    msg->append(lines[i] + "\n");
                    cout << "MSG: " << (*msg.get());
                    this->callback(msg);
                }

                buffer.consume(length);
            }
        }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    this->start_accept_handler();

    cout << "Socket disconnected\n";
}

void OacSubscriber::start_accept_handler()
{
    boost::shared_ptr<boost::asio::ip::tcp::socket> socket(new boost::asio::ip::tcp::socket(this->accept_service));
    this->acceptor->async_accept(*socket, boost::bind(&OacSubscriber::handle_accept, this, socket));
}

void OacSubscriber::run_accept_service_thread()
{
    this->accept_service.run();
}
