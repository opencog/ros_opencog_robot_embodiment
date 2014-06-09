#include "OacSubscriber.h"

using namespace ros_opencog_robot_embodiment;
using namespace std;

OacSubscriber::OacSubscriber(int local_port)
{
    this->local_port = local_port;
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
        ROS_DEBUG("OacSubscriber started.");
    }
    else
    {
        ROS_WARN("OacSubscriber already started.");
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
    cout << "Socket connected\n";//buffer consume


    try
    {
        boost::asio::streambuf buffer;

        while(true)
        {
            boost::system::error_code err;
            boost::asio::read_until(*socket, buffer, "\n", err);

            if (err)
            {
                std::cerr << "Error: " <<  err << "\n";
                break;
            }
            else
            {
                //boost::shared_ptr<string> line(new string(""));
                std::istream stream(&buffer);

                int read_size = 0;
                while(true)
                {
                    std::string line;
                    std::getline(stream, line);

                    if(stream.eof())
                    {
                        break;
                    }
                    else
                    {
                        cout << "Read line: " << line << "\n";
                        this->process_line(line + '\n');
                        read_size += line.size();
                    }
                }

                buffer.consume(read_size);
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

//boost::shared_ptr<std::string> msg

void OacSubscriber::process_line(string line)
{
    std::ofstream file;
    file.open("/media/AA32AB9032AB5FD5/opencog_data/oac_output.txt", ios::app);

    if (file.is_open())
    {
        cout << "Writing oac output...\n";

        file << line;
        file.close();
    }
    else
    {
        cout << "Error file not open.";// << path;
    }

    char selector = line[0];

    if(selector == 'c')
    {
        string command = line.substr(1);
        this->process_oac_command(command);
    }
    else if(selector == 'd')
    {
        string data = line.substr(1);
        this->current_msg << data;
    }
    else
    {
        ROS_WARN("Unknown line prefix: %s", "" + selector);
    }
}

void OacSubscriber::process_oac_command(std::string command)
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
            //this->is_router_available = true;
        }
    }
    else if(name == "UNAVAILABLE_ELEMENT")
    {
        string id = tokens[1];

        if(id == "ROUTER")
        {
            //this->is_router_available = false;
        }
    }
    else if(name == "START_MESSAGE")
    {
        cout << "Start message\n";
        this->current_msg.str(string());
    }
    else if(name == "NO_MORE_MESSAGES")
    {
        this->fire_message_callbacks(this->current_msg.str());
        cout << "Finish message\n";
    }
    else
    {
        cout << "Unsupported message\n";
    }
}

void OacSubscriber::register_oac_loaded_callback(OacLoadedCallback oac_loaded_callback)
{
    this->oac_loaded_callback = oac_loaded_callback;
}

void OacSubscriber::register_action_plan_callback(ActionPlanCallback action_plan_callback)
{
    this->action_plan_callback = action_plan_callback;
}

void OacSubscriber::fire_message_callbacks(string message)
{
    //cout << "Message: \"" << message << "\"";

    if(boost::starts_with(message, Commands::SUCCESS_LOAD))
    {
        //cout << "Agent loaded.\n";

        if(this->oac_loaded_callback)
        {
            boost::thread(boost::bind(this->oac_loaded_callback, true));
        }
        else
        {
            ROS_ERROR("oac_loaded_callback not set");
        }
    }
    else
    {
        TiXmlDocument doc;
        bool loaded = doc.Parse(message.c_str());

        if(loaded)
        {
            TiXmlElement *emotional_feeling_element = doc.FirstChildElement("oc:emotional-feeling");
            TiXmlElement *action_plan_element = doc.FirstChildElement("oc:action-plan");

            if(emotional_feeling_element)
            {
                ROS_DEBUG("oc:emotional-feeling received, discarding");
            }
            else if(action_plan_element)
            {

                ActionPlan action_plan = ActionPlan::from_xml(message);
                ROS_DEBUG("ActionPlan received");

                if(this->action_plan_callback)
                {
                    boost::thread(boost::bind(this->action_plan_callback, action_plan));
                }
                else
                {
                    ROS_ERROR("action_plan_callback not set");
                }
            }
            else
            {
                ROS_ERROR("Invalid message type: %s", message.c_str());
            }
        }
        else
        {
            ROS_ERROR("Invalid XML: %s", message.c_str());
        }
    }
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
