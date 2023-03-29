#include "MQTTCommunication.h"
#include <unistd.h>

int main(int argc, char* argv[])
{
    std::string arr[] = {"apple", "banana", "orange", "grape"};
    std::string target = "banana";
    auto it = std::find(std::begin(arr), std::end(arr), target);
    if (it != std::end(arr))
        std::cout << "Found " << target << " in the array." << std::endl;
    else
        std::cout << "Did not find " << target << " in the array." << std::endl;


	state_machine rsm;

	mqtt::async_client cli(SERVER_ADDRESS,CLIENT_ID);

    auto connOpts = mqtt::connect_options_builder()
                    .clean_session(true)
                    .automatic_reconnect(std::chrono::seconds(3), std::chrono::seconds(10))
                    .finalize();

	callback cb(cli,connOpts,&rsm);
	cli.set_callback(cb);
	cli.connect(connOpts,nullptr,cb);
    while(!cli.is_connected())
    {
        try
        {
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
        catch(const mqtt::exception& exc)
        {
            std::cout<<exc.what()<<std::endl;
        }
    }

	while(true)
	{
		rsm.run();
		sleep(1);
	}

	// while(std::tolower(std::cin.get())!='q');


}