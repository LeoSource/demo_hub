#include <iostream>
#include <fstream>
#include <string>
#include "yaml.h"


int main(int, char**) 
{
    std::string g_mqtt_address = "tcp://1883";
    std::string g_client_id = "hr_robot";
    double g_encoder_resolution[] = {0,0,0,0,0};
    double g_encoder_offset[] = {0,0,0,0,0};
    double g_encoder_dir[] = {0,0,0,0,0};
    double g_gear_ratio[] = {0,0,0,0,0};
    double g_mechanism_limit[] = {0,0,0,0,0};
    double g_manu_vmax[] = {0.00,0.00,0.00,0.00,0.0};
    double g_manu_acc[] = {0.0,0.0,0.00,0.00,0.0};

    try
    {
        YAML::Node config = YAML::LoadFile("../config_parameter.yaml");
        g_mqtt_address = config["mqtt_address"].as<std::string>();
        for(int idx=0;idx<config["encoder_offset"].size();idx++)
        {
            g_encoder_resolution[idx] = config["encoder_resolution"].as<std::vector<double>>()[idx];
            g_encoder_offset[idx] = config["encoder_offset"].as<std::vector<double>>()[idx];
            g_encoder_dir[idx] = config["encoder_dir"].as<std::vector<double>>()[idx];
            g_manu_vmax[idx] = config["manu_vmax"].as<std::vector<double>>()[idx];
            g_manu_acc[idx] = config["manu_acc"].as<std::vector<double>>()[idx];
        }
    }
    catch(YAML::ParserException &ex)
    {
        std::cerr << ex.what() << '\n';
    }
    catch(const std::exception& ex)
    {
        std::cerr << ex.what() << '\n';
    }

    



    // double array_data[] = {0,0,0,0};

    // YAML::Node config = YAML::LoadFile("../config.yaml");
    // std::cout << "name: " <<config["name"].as<std::string>()<<std::endl;
    // std::cout<<"age: "<<config["age"].as<int>()<<std::endl;

    // std::cout<<"skills-c++: "<<config["skills"]["c++"].as<int>()<<std::endl;
    // auto array = config["array"];
    // std::cout<<"type: " <<array.Type() <<std::endl;
    // std::cout<<"array0: "<< array[0]<<std::endl;
    // std::cout<<"array size: "<<array.size()<<std::endl;
    // for(int idx=0;idx<config["array"].size();idx++)
    // {
    //     array_data[idx] = config["array"].as<std::vector<double>>()[idx];
    // }
    // // std::count<<"array: "<< config["array"].as<std::vector<double>>()[2] <<std::endl;

    // YAML::Node test1 = YAML::Load("[1,2,3,4]");
    // std::cout<<"type: "<<test1.Type()<<std::endl;
    // std::cout<<"test_array: "<< test1.as<std::vector<double>>()[0] <<std::endl;
    return 0;
}
