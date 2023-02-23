#include "MQTTCommunication.h"

void action_listener::on_failure(const mqtt::token& tok)
{
    if (tok.get_message_id() != 0)
        std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
    std::cout << std::endl;
}

void action_listener::on_success(const mqtt::token& tok)
{
    std::cout << name_ << " success";
    if (tok.get_message_id() != 0)
        std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
    auto top = tok.get_topics();
    if (top && !top->empty())
        std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
    std::cout << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void callback::reconnect()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    try {
        cli_.connect(connOpts_, nullptr, *this);
    }
    catch (const mqtt::exception& exc) {
        std::cerr << "Error: " << exc.what() << std::endl;
        exit(1);
    }
}

void callback::on_failure(const mqtt::token& tok)
{
    std::cout << "Connection attempt failed" << std::endl;
    if (++nretry_ > N_RETRY_ATTEMPTS)
        exit(1);
    reconnect();
}

void callback::connected(const std::string& cause)
{
    std::cout << "\nConnection success" << std::endl;
    std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
        << "\tfor client " << CLIENT_ID
        << " using QoS" << QOS << "\n"
        << "\nPress Q<Enter> to quit\n" << std::endl;

    cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
}

void callback::connection_lost(const std::string& cause)
{
    std::cout << "\nConnection lost" << std::endl;
    if (!cause.empty())
        std::cout << "\tcause: " << cause << std::endl;

    std::cout << "Reconnecting..." << std::endl;
    nretry_ = 0;
    reconnect();    
}

void callback::message_arrived(mqtt::const_message_ptr msg)
{
    std::cout << "Message arrived" << std::endl;
    std::cout << "topic: '" << msg->get_topic() << "'" << std::endl;
    auto json_str = json::parse(msg->to_string());
    // json_str.at("actual_position").get_to(_jpos);
    // std::cout<<_jpos[0]<<std::endl;
    // std::cout<<_jpos[1]<<std::endl;
    // std::cout<<_jpos[2]<<std::endl;
    // std::cout<<_jpos[3]<<std::endl;
    // std::cout<<_jpos[4]<<std::endl;
    // std::cout<<_jpos[5]<<std::endl;
    std::cout << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;    
}

