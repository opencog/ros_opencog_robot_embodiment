#include "OacServer.h"

using namespace ros_opencog_robot_embodiment;
using namespace std;
using namespace boost;

OacSubscriber::OacSubscriber(boost::function<void(boost::shared_ptr<string>)> callback)
{
    this->callback = callback;
    this->reading = false;
}

OacSubscriber::~OacSubscriber()
{
    this->stop();
}

void OacSubscriber::read_data()
{
    /*boost::asio::io_service io_service;

    boost::asio::ip::tcp::resolver resolver(io_service);
    //boost::asio::ip::tcp::resolver::query query("localhost", "daytime");
    boost::asio::ip::tcp::resolver::query query("localhost", boost::lexical_cast<string>(12000));
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    boost::asio::ip::tcp::socket socket(io_service);

    boost::system::error_code error = boost::asio::error::host_not_found;
    while (error && endpoint_iterator != end)
    {
         std::cerr << "hi\n";
        socket.close();
        //boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator++;
        socket.connect(*endpoint_iterator++, error);
        //this->port = endpoint.port();
    }*/

    boost::asio::io_service io_service;
    //boost::asio::ip::tcp::acceptor acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 12000));
    boost::asio::ip::tcp::socket socket(io_service);
    socket.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 12001));
    //acceptor.accept(socket);

    std::cerr << "hi\n";

    //if (error)
   // {
        //throw boost::system::system_error(error);
   // }

    this->port = 12000;

    try
    {
        boost::asio::streambuf buffer;

        while(this->reading)
        {
            typedef boost::asio::streambuf::const_buffers_type buffer_type;
            typedef boost::asio::buffers_iterator<buffer_type> iterator;
            boost::system::error_code err;
            int length = boost::asio::read_until(socket, buffer, "\n", err);

            if (err)
            {
                std::cerr << "shit happened...\n";
            }
            else
            {
                cout << "Read line successfully...\n";
                boost::shared_ptr<string> line(new string(""));
                buffer_type input = buffer.data();
                std::copy(iterator::begin(input), iterator::end(input), std::back_inserter(*line));
                this->callback(line);
                buffer.consume(length);
            }
        }

     }
     catch (std::exception& e)
     {
       std::cerr << e.what() << std::endl;
     }
}

void OacSubscriber::start()
{
    if(!this->reading)
    {
        this->reading = true;
        boost::thread(&OacSubscriber::read_data, this);
    }
}

int OacSubscriber::get_port()
{
    return this->port;
}

void OacSubscriber::stop()
{
    if(this->reading)
    {
        this->reading = false;
    }
}
