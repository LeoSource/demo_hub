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
    auto j = json::parse(msg->to_string());
	auto is_type_valid = [](const std::string& type_name)->bool {
		std::vector<std::string> type_set{"master","joint","posrpy","rpy","pose","direction"};
		return std::count(type_set.begin(),type_set.end(),type_name)!=0;};
	auto calc_rows = [is_type_valid](const std::string& type_name)->int {
		std::map<std::string,int> type_rows{{"master",3},{"joint",5},{"posrpy",5},{"rpy",2},{"pose",6},{"direction",3}};
		return is_type_valid(type_name)?type_rows[type_name]:-1;};
    auto check_rows = [j](const std::string& part,int rows)->bool {
        bool valid = true;
        std::vector<std::vector<double>> pos = j[part]["pos"];
        for(auto it=pos.begin();it!=pos.end();it++)
            valid = valid && (it->size()==rows);
        return valid;
    };
    if(j.contains("slave") && j["slave"].contains("type"))
	{
		auto is_valid = [j,check_rows](int rows)->bool
		{
			auto slave = j["slave"];
			bool valid = slave.contains("pos") && slave.contains("relative") && slave.contains("joint-space");
			if(slave["type"]!="joint")
				valid = valid && slave.contains("rcm-constraint");
            return valid?check_rows("slave",rows):false;
		};
        int rows = calc_rows(j["slave"]["type"]);
		if(is_valid(rows))
		{
            std::cout<<"slave motion arguments is valid."<<std::endl;
            std::vector<std::vector<double>> pos = j["slave"]["pos"];
            Eigen::MatrixXd via_pos(rows,pos.size());
            for(int idx=0;idx<pos.size();idx++)
            {
                Eigen::VectorXd vp = Eigen::Map<Eigen::VectorXd>(pos[idx].data(),pos[idx].size());
                // via_pos.col(idx) = j["slave"]["relative"]?vp+static_cast<Eigen::VectorXd>(_prs_fdb.jpd.pos):vp;
            }
            double vel_ratio = j.contains("vel_ratio")?static_cast<double>(j["vel_ratio"]):1.0;
            double acc_ratio = j.contains("acc_ratio")?static_cast<double>(j["acc_ratio"]):1.0;
            // _splanner->reset(_prs_fdb.jpd.pos);
            // auto traj_type = j["slave"]["joint-space"]?slave_traj_type::e_joint:slave_traj_type::e_cartesian;
            // _splanner->add_traj(via_pos,traj_type,vel_ratio,acc_ratio);
            // return state_name::e_motion;
		}
		else
        {
            std::cout<<"salve motion arguments is invalid!!"<<std::endl;
            // return state_name::e_none;
        }
	}


    std::vector<std::vector<double>> pos_vec = j["slave"]["pos"];
    Eigen::MatrixXd pos_eig(5,pos_vec.size());
    for(int idx=0;idx<pos_vec.size();idx++)
        pos_eig.col(idx) = Eigen::Map<Eigen::VectorXd>(pos_vec[idx].data(),pos_vec[idx].size());
    std::cout<<pos_eig<<std::endl;
    std::cout << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;    
    if(_rsm->handle_event(j.at("name")))
        std::cout<<"action "<<j.at("name")<<" has been executed"<<std::endl;
    else
        // std::cout<<static_cast<typename std::underlying_type<state_name>::type>(_rsm->get_cur_state_name())<<std::endl;
        std::cout<<"current state "<<
            static_cast<typename std::underlying_type<state_name>::type>(_rsm->get_cur_state_name())
            <<" cannot be response for action: "<<j.at("name")<<std::endl;
}

