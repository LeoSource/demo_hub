#include "MQTTCommunication.h"

int main(int argc, char* argv[])
{

	mqtt::async_client cli(SERVER_ADDRESS,CLIENT_ID);

	mqtt::connect_options connOpts;
	connOpts.set_clean_session(false);

	callback cb(cli,connOpts);
	cli.set_callback(cb);

	// MQTTTransition mqtt_trans(cli,connOpts,cb);
	// mqtt_trans.Connect();

	while(std::tolower(std::cin.get())!='q');

	// mqtt_trans.DisConnect();

}