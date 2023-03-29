#pragma once

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <atomic>
#include "mqtt/async_client.h"
#include "json.hpp"
#include "robot_state.h"

using nlohmann::json;

// const std::string SERVER_ADDRESS("tcp://192.168.3.33:1883");
const std::string SERVER_ADDRESS("tcp://broker.emqx.io");
const std::string CLIENT_ID("hr_robot");
const std::string TOPIC("fsm");
const int QOS = 1;
const int N_RETRY_ATTEMPTS = 5;

class action_listener : public virtual mqtt::iaction_listener
{
	std::string name_;

	void on_failure(const mqtt::token& tok) override;

	void on_success(const mqtt::token& tok) override;

public:
	action_listener(const std::string& name) : name_(name) {}
};

////////////////////////////////////////////////////////////////////////////////

class callback : public virtual mqtt::callback,
					public virtual mqtt::iaction_listener
{
	// Counter for the number of connection retries
	int nretry_;
	// The MQTT client
	mqtt::async_client& cli_;
	// Options to use if we need to reconnect
	mqtt::connect_options& connOpts_;
	// An action listener to display the result of actions.
	action_listener subListener_;

	state_machine* _rsm;

	// This deomonstrates manually reconnecting to the broker by calling
	// connect() again. This is a possibility for an application that keeps
	// a copy of it's original connect_options, or if the app wants to
	// reconnect with different options.
	// Another way this can be done manually, if using the same options, is
	// to just call the async_client::reconnect() method.
	void reconnect();

	// Re-connection failure
	void on_failure(const mqtt::token& tok) override;

	// (Re)connection success
	// Either this or connected() can be used for callbacks.
	void on_success(const mqtt::token& tok) override {}

	// (Re)connection success
	void connected(const std::string& cause) override;

	// Callback for when the connection is lost.
	// This will initiate the attempt to manually reconnect.
	void connection_lost(const std::string& cause) override;

	// Callback for when a message arrives.
	void message_arrived(mqtt::const_message_ptr msg) override;

	void delivery_complete(mqtt::delivery_token_ptr token) override {}

public:
	callback(mqtt::async_client& cli, mqtt::connect_options& connOpts,state_machine* rsm)
				: nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription"),_rsm(rsm) {}
};



